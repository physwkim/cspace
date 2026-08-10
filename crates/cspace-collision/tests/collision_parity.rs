// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `collision` op.
//!
//! Ground truth is the oracle's own response, captured verbatim into
//! `tests/fixtures/{panda,fanuc,pr2}_collision.json`: one default case
//! (`joint_values: {}`) plus three cases sampled by the oracle's own
//! `random_states` op, exactly `fk_parity.rs`'s pattern. Every case checks the
//! robot against one fixed world object -- a `4x4x0.1` floor box centered at
//! `(0, 0, -0.05)`, so its top face sits at `z = 0` -- built via
//! [`ParryCollisionEnv::check_self_collision`]/[`check_robot_collision`]/
//! [`distance_self`]/[`distance_robot`] with `enable_signed_distance: true`.
//!
//! `cspace_core::model::LinkModel` now loads `<mesh>` collision geometry (STL,
//! resolved through [`MeshSearchPaths`] -- see that type and
//! `cspace_core::geometry`'s `stl` module), so panda and fanuc (whose collision
//! geometry is exactly one `<mesh>` element per link) and pr2's `<mesh>`
//! links build with real collision shapes here rather than none at all.
//! [`fixture_mesh_search_paths`] points at the three packages committed
//! under `fixtures/meshes/`.
//!
//! Contact-point/nearest-point coordinates are never compared here.
//! PORTING-PLAN.md §4.5 records that exclusion as Phase 3's recorded
//! verification limit, not an oversight: `parry.rs`'s own module doc
//! (deviations 4 and 6) establishes that this backend's contact geometry
//! differs from FCL's by construction (at most one contact per pair, taken
//! from a single `parry3d_f64::query::contact` call, versus FCL's up to 200
//! contacts per pair) in ways that cannot converge under any tolerance.
//!
//! # No exact penetration-depth parity for interpenetrating meshes
//!
//! With real mesh geometry loaded, this is the first ground truth exercising
//! `parry.rs`'s deviation 6 in the actually-penetrating regime it describes:
//! upstream's `distanceCallback`, once a pair is confirmed touching or
//! penetrating, re-runs `fcl::collide` (up to 200 contacts) and takes the
//! *maximum* penetration depth found; this backend's single
//! `parry3d_f64::query::contact` call returns exactly one (not necessarily
//! the same) contact for the whole pair. For two convex primitives that
//! never differ -- there is only one contact to find -- but for a mesh
//! overlapping another shape across many triangles, the two independent EPA
//! implementations (`parry3d_f64`'s and FCL/libccd's) are not guaranteed to
//! settle on the same local penetration, and the two numbers do not converge
//! under any tolerance.
//!
//! Confirmed at three scales, and not always by the same mechanism. Two of
//! `fanuc_collision.json`'s self-colliding cases (mesh vs. mesh) -- case 1
//! (`link_1`/`link_4`): oracle `self_distance = -0.01624`, this backend
//! `-0.00561`; case 2 (`link_1`/`link_5`): oracle `-0.07129`, this backend
//! `-0.02322` -- and a live `tools/moveit-diff --collision` sweep of
//! `panda_arm` (mesh vs. the floor's `<box>`) both disagree on a *shared*
//! pair's depth, oracle deeper each time (e.g. oracle `-1.896`, this backend
//! `-0.149` on one panda case). [`pr2_case_7552_depth_disagreement_ranks_a_different_pair`]
//! is a third case, and the mechanism there is one level up: pr2's case 7552
//! has *several* simultaneously-interpenetrating mesh pairs on both the self
//! and robot side, each with its own independently-computed depth, and this
//! backend and the oracle each rank a *different* pair as globally deepest
//! (self: oracle picks `l_gripper_r_finger_link`/`l_gripper_palm_link` at
//! `-0.03027`, this backend picks `base_bellow_link`/`torso_lift_link` at
//! `-0.05293`; robot: oracle picks `r_gripper_l_finger_link`/`floor` at
//! `-0.02488`, this backend picks `r_gripper_l_finger_tip_link`/`floor` at
//! `-0.05094`). Isolating each side's own pick shows *why*: this backend's
//! own answer for the oracle's self pick is `-0.00188`, far shallower than
//! its own `-0.05293` winner, so it never had a chance to win; its own answer
//! for the oracle's robot pick is `-0.02833`, close to the oracle's `-0.02488`
//! for that same pair, so the robot-side disagreement is almost entirely
//! about *which finger link* wins, not about the winning depth. Every
//! *non*-penetrating case (every fanuc/panda/pr2 case with `robot_collision:
//! false`) still agrees to `~1e-9`-`~1e-16`, since there only one contact
//! exists for either algorithm to find. [`assert_full_parity_matches_oracle`]
//! therefore asserts full distance-magnitude parity only when the oracle
//! reports no collision on that side; when it reports a collision, only the
//! sign (`<= TOLERANCE`) is asserted, matching what the boolean
//! `self_collision`/`robot_collision` check already independently confirms.
//! A bare sign check would tolerate any magnitude at all, including a future
//! regression to panda's own impossible-number failure mode above, so every
//! colliding case also gets [`assert_plausible_depth`]: twice the reported
//! pair's own smaller bounding radius, computed by [`link_bounding_radius`]
//! generically over every [`Shape`] kind (not just [`Shape::Mesh`] --
//! `base_bellow_link`, below, is a `<box>`, and a mesh-only bound would
//! silently read as a 0m radius for it).
//!
//! `base_bellow_link`/`torso_lift_link` (case 7552's self pick, above) is a
//! fourth confirmed instance of this same deviation, and the clearest one:
//! it is this backend's own dominant self-collision constant across the
//! seed-1, 10,000-case pr2 sweep, and it is the one pair on all of pr2 whose
//! relative pose is a pure function of a single joint
//! (`torso_lift_joint`) -- so its whole plateau/ramp shape over that joint's
//! range is directly inspectable, not just spot-checked at one state. This
//! backend's own curve there is `min(candidate_x, candidate_z(t))`:
//! `candidate_x` is a `torso_lift_joint`-invariant `-x`-face-vs-mesh planar
//! contact, and `candidate_z(t)` is a genuinely `t`-dependent z-direction
//! candidate, linear in `t` with slope `1`. Below the crossover `candidate_x`
//! is shallower and this backend reports the plateau; the oracle's own sweep
//! over the same range instead decreases smoothly and monotonically at very
//! nearly `1:1` with the joint travel -- a real z-direction overlap, not
//! noise -- so "the oracle's own answer fails to hold constant" is not
//! itself the argument (a claim that the true depth *cannot* change over
//! that span would be circular, assuming the very thing in question). Past
//! the crossover `candidate_z(t)` shallows further and wins, and this
//! backend's answer should then match the oracle's, not merely resemble it.
//! [`pr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates`]
//! asserts exactly that: it establishes both candidates from live samples,
//! bisects this backend's own observed crossover independently of the fitted
//! lines, confirms the two candidates actually meet there to within this
//! crate's own `TOLERANCE`, and checks agreement with a real captured oracle
//! response at `torso_lift_joint = 0.22`, past the crossover, to `~1e-9`. See
//! `parry.rs`'s deviation 6 for the generalization from mesh-vs-mesh to
//! convex-primitive-vs-mesh this licenses, and for the two pair-families
//! that made this deviation look like three frozen constants at first.
//!
//! That the divergence is *confined to* the interpenetrating regime was
//! confirmed above; that deviation 6 is *why* it diverges there was, until
//! [`panda_worst_sweep_deviation_is_not_a_missed_deeper_contact`], an
//! unfalsified argument, not a tested one. That test picks the sweep's
//! single worst case (case 3289, `|d| = 2.738`) and shows the argument does
//! not hold for it: `parry3d_f64::query::contact` against a mesh already
//! visits every triangle overlapping the other shape and keeps the deepest
//! (`contact_composite_shape_shape`, read from the vendored crate source --
//! see that test's doc comment), so there is no unexploited
//! maximum-over-contacts left for this backend to add, and the colliding
//! link's own mesh is geometrically too small (bounding radius well under a
//! meter) for a `2.8`-unit penetration depth to be physically possible
//! against it at all. For *this* case, the oracle's own FCL/libccd
//! penetration-depth computation is the one producing an impossible number,
//! not this backend missing a deeper real contact -- a distinct,
//! upstream-side numerical failure mode (deep, arbitrarily-rotated
//! interpenetration defeating libccd's EPA), not an instance of deviation 6
//! at all. Deviation 6 itself is confirmed instead by
//! [`pr2_case_7552_depth_disagreement_ranks_a_different_pair`]: `|d|`s two
//! orders of magnitude smaller (`~0.02`-`0.03`), well inside the relevant
//! links' own bounding radii, consistent with independent EPA implementations
//! disagreeing on the deepest point among several genuinely-overlapping mesh
//! pairs -- and, when several such disagreements are close in magnitude,
//! disagreeing on *which pair* is deepest at all -- rather than either one
//! hallucinating an impossible number. That pair-ranking instability is
//! distinct from the visibility_cone `max_contacts: 1` traversal-order
//! tie-break (round-8 §38.3): that one picks a first-found pair out of a
//! *truncated, non-exhaustive* contact query, where this backend's
//! `distance_self`/`distance_robot` never truncate (deviation 7) -- every
//! pair here really was compared, so the ranking disagreement reflects real,
//! if unstable, per-pair depth disagreement rather than which pair a
//! budget-limited search happened to visit first. Whether deviation 6 also
//! explains the sweep's other ~9,500 distance-only disagreements is not
//! claimed -- each would need its own such check -- so this narrows
//! deviation 6 from "the explanation" to "an explanation confirmed for
//! modest-magnitude interpenetration (same pair or a close competitor), and
//! demonstrably not the cause of the sweep's own worst (and geometrically
//! impossible) outlier."
//!
//! # pr2's mesh gap was a fixture gap, not a feature gap
//!
//! Earlier rounds asserted only `robot_collision`/`robot_distance` for pr2:
//! a live 10,000-state sweep against a rust build with pr2's real `<mesh>`
//! collision links unresolved (no committed fixture mesh tree for pr2, only
//! the leftover `<box>`/`<cylinder>`/`<sphere>` links) found `self_collision`
//! disagreeing in 9,999 of 10,000 cases, `self_distance` pinned near a single
//! pose-invariant `~2.9 cm` (the leftover primitive pair's own separation)
//! while the oracle's real mesh-driven self-collision varied per state.
//! [`fixture_mesh_search_paths`] now points pr2 at
//! `fixtures/meshes/pr2_description/` the same way panda/fanuc already were,
//! and [`pr2_collision_matches_the_oracle`] asserts the same full
//! `self_collision`/`robot_collision` parity panda and fanuc do.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use cspace_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, DistanceResultsData,
    LinkPaddingScale, ParryCollisionEnv, World,
};
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

#[derive(Deserialize)]
struct CollisionCase {
    joint_values: BTreeMap<String, f64>,
    self_collision: bool,
    self_distance: f64,
    robot_collision: bool,
    robot_distance: f64,
}

#[derive(Deserialize)]
struct CollisionFixture {
    cases: Vec<CollisionCase>,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn load_fixture(file_name: &str) -> CollisionFixture {
    let path = fixture_path(file_name);
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

/// One request from `pr2_torso_lift_bellow_sweep_request.json` -- the raw
/// oracle wire shape (`{"id", "joint_values", "objects", "op"}`), not the
/// pre-flattened [`CollisionCase`] shape the other fixtures use, since this
/// one is read alongside its own request to match request/response by `id`
/// rather than array position (`verify-fixture-replay.sh`'s own rule).
#[derive(Deserialize)]
struct TorsoSweepRequestCase {
    id: u64,
    joint_values: BTreeMap<String, f64>,
}

#[derive(Deserialize)]
struct TorsoSweepDistancePair {
    body_name_1: String,
    body_name_2: String,
}

#[derive(Deserialize)]
struct TorsoSweepResult {
    self_distance: f64,
    self_distance_pair: TorsoSweepDistancePair,
}

#[derive(Deserialize)]
struct TorsoSweepResponseCase {
    id: u64,
    result: TorsoSweepResult,
}

/// One `torso_lift_joint` state and the oracle's own `self_distance` for
/// `base_bellow_link`/`torso_lift_link` there, read from
/// `pr2_torso_lift_bellow_sweep_{request,response}.json` -- a real captured
/// oracle response (round 10 §56.3/§40), not a transcribed literal.
struct TorsoSweepOraclePoint {
    torso_lift_joint: f64,
    self_distance: f64,
}

fn load_torso_sweep_oracle_points() -> Vec<TorsoSweepOraclePoint> {
    let requests: Vec<TorsoSweepRequestCase> = {
        let path = fixture_path("pr2_torso_lift_bellow_sweep_request.json");
        let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
    };
    let responses: Vec<TorsoSweepResponseCase> = {
        let path = fixture_path("pr2_torso_lift_bellow_sweep_response.json");
        let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
    };
    let responses_by_id: BTreeMap<u64, &TorsoSweepResponseCase> =
        responses.iter().map(|r| (r.id, r)).collect();

    requests
        .iter()
        .map(|req| {
            let torso_lift_joint = *req
                .joint_values
                .get("torso_lift_joint")
                .expect("torso sweep fixture request must set torso_lift_joint");
            let response = responses_by_id
                .get(&req.id)
                .unwrap_or_else(|| panic!("no response for request id {}", req.id));
            let pair = &response.result.self_distance_pair;
            assert!(
                (pair.body_name_1 == "torso_lift_link" && pair.body_name_2 == "base_bellow_link")
                    || (pair.body_name_1 == "base_bellow_link"
                        && pair.body_name_2 == "torso_lift_link"),
                "request id {}: oracle's own global self_distance winner there is {}/{}, not \
                 torso_lift_link/base_bellow_link -- this fixture's states must stay inside the \
                 range where this pair is the oracle's own global argmin",
                req.id,
                pair.body_name_1,
                pair.body_name_2
            );
            TorsoSweepOraclePoint {
                torso_lift_joint,
                self_distance: response.result.self_distance,
            }
        })
        .collect()
}

/// The three `moveit_resources_*_description` packages committed under
/// `fixtures/meshes/` (see `tools/ci/verify-fixture-provenance.sh`).
fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([
        (
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        ),
        (
            "moveit_resources_fanuc_description",
            format!("{meshes_root}/fanuc_description"),
        ),
        (
            "moveit_resources_pr2_description",
            format!("{meshes_root}/pr2_description"),
        ),
    ])
}

fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        urdf_file
    );
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let urdf_xml = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(&path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("fixture model must build")
}

/// The oracle's `collision` op checks against
/// `AllowedCollisionMatrix(*model_->getSRDF())` (`buildAcm` in `oracle.cpp`),
/// so every case in the fixtures was captured with fanuc/pr2's
/// `disable_collisions` entries suppressing the pairs they name. Without
/// applying the same matrix here, this test would disagree with the oracle
/// on exactly those suppressed pairs -- not a `ParryCollisionEnv` defect, a
/// missing input.
fn build_acm(srdf_file: &str) -> AllowedCollisionMatrix {
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// The same `4x4x0.1` floor box, at the same pose, the oracle fixtures were
/// captured against (see the module doc). Built once per test the same way
/// `tools/moveit-diff`'s own `collision_scene` is, so both crates' tests bear
/// out the identical geometry that comparison relies on.
fn floor_env() -> ParryCollisionEnv {
    floor_env_with_top(0.0)
}

/// [`floor_env`] with the box's *top face* placed at `top_z`, the same
/// parameter `tools/moveit-diff --floor-top-z` names and with the same meaning
/// -- the face the robot base stands on, not the box centre.
///
/// A parameter rather than a second hard-coded constructor: the two scenes
/// differ only in that one number, and a copy would let the box size or
/// thickness drift apart between them, which is exactly the input every
/// closed-form claim in this file is derived from.
fn floor_env_with_top(top_z: f64) -> ParryCollisionEnv {
    const FLOOR_THICKNESS: f64 = 0.1;
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, FLOOR_THICKNESS)
                .expect("4x4x0.1 are valid, positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, top_z - FLOOR_THICKNESS / 2.0),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

fn build_state<'m>(model: &'m RobotModel, joint_values: &BTreeMap<String, f64>) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    state
}

/// `1e-4`, per PORTING-PLAN.md §5's distance tolerance for Phase 3's
/// completion condition.
const TOLERANCE: f64 = 1e-4;

/// An upper bound on how far any of `link_name`'s own collision geometry can
/// reach from that link's local origin -- a rigid body cannot overlap
/// anything by more than its own diameter, in any orientation, at any pose.
/// `None` for a name that does not resolve to a link at all (e.g. `"floor"`,
/// a [`World`] object), or whose only shapes have no finite extent
/// ([`Shape::Plane`], [`Shape::OcTree`]) -- callers fall back to whichever
/// side of a pair does resolve.
///
/// Generalizes the `Shape::Mesh`-only bound
/// `panda_worst_sweep_deviation_is_not_a_missed_deeper_contact` and
/// `pr2_case_7552_depth_disagreement_ranks_a_different_pair` each hand-roll
/// for their one mesh-shaped link of interest: `base_bellow_link` (see
/// `pr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates`,
/// below) is a `<box>`,
/// not a mesh, and a Mesh-only bound would silently read as a 0m radius --
/// falsely making any nonzero depth for that pair look impossible instead of
/// plausible.
///
/// For a robot-link/[`World`]-object pair this bound is one-sided by
/// construction: the world side always resolves to `None` (it is never a
/// `link_name` `model.link_model` can find), so `assert_plausible_depth`'s
/// `min_by` has only the robot link's own radius to work with, and the
/// bound degenerates to "twice this one link's own radius" regardless of
/// how large or small the world object is. For
/// `pr2_world_object_same_pair_deeper_depth_is_a_real_vertex_not_a_spurious_direction`'s
/// two cases that bound is `2 * 0.0346m ~= 0.069m` -- comfortably above
/// both the oracle's ~0.010-0.011m and this backend's ~0.012-0.016m, so it
/// cannot distinguish between them. `assert_plausible_depth` was never the
/// mechanism that would have caught that disagreement; only an independent
/// measurement against the actual world geometry (`deepest_vertex_under_floor`)
/// can.
fn link_bounding_radius(model: &RobotModel, link_name: &str) -> Option<f64> {
    let link = model.link_model(link_name).ok()?;
    link.shapes().iter().try_fold(0.0_f64, |acc, link_shape| {
        let local_radius = match &link_shape.shape {
            Shape::Sphere(s) => s.radius,
            Shape::Cuboid(b) => b.size[0].hypot(b.size[1]).hypot(b.size[2]) * 0.5,
            Shape::Cylinder(c) => c.radius.hypot(c.length * 0.5),
            Shape::Cone(c) => c.radius.hypot(c.length * 0.5),
            Shape::Mesh(mesh) => mesh
                .vertices
                .iter()
                .map(|v| nalgebra::Point3::from(*v).coords.norm())
                .fold(0.0_f64, f64::max),
            Shape::Plane(_) | Shape::OcTree(_) => return None,
        };
        Some(acc.max(link_shape.origin_transform.translation.vector.norm() + local_radius))
    })
}

/// The companion round 9 item 1 asked for: `assert_full_parity_matches_oracle`
/// only checks the *sign* of a colliding pair's depth (module doc, "no exact
/// penetration-depth parity"), and a bare sign check tolerates literally any
/// magnitude -- it would not catch this backend regressing to a
/// panda-worst-case-style impossible number. This does not assert *parity*
/// (the whole point of the sign-only branch is that the two backends'
/// magnitudes are not required to converge); it asserts *plausibility*: twice
/// the reported pair's own smaller bounding radius, the same yardstick
/// `panda_worst_sweep_deviation_is_not_a_missed_deeper_contact` and
/// `pr2_case_7552_depth_disagreement_ranks_a_different_pair` already use by
/// hand. Silently skipped when neither named link resolves to a bounded
/// shape (e.g. both sides are non-robot [`World`] objects, or either is a
/// [`Shape::Plane`]/[`Shape::OcTree`]) -- there is no yardstick to check
/// against in that case, not a pass by default.
fn assert_plausible_depth(
    model: &RobotModel,
    fixture_name: &str,
    case_index: usize,
    field_name: &str,
    minimum_distance: &DistanceResultsData,
) {
    let bound = minimum_distance
        .link_names
        .iter()
        .filter_map(|name| link_bounding_radius(model, name))
        .min_by(f64::total_cmp)
        .map(|radius| 2.0 * radius);
    if let Some(bound) = bound {
        assert!(
            minimum_distance.distance.abs() <= bound,
            "{fixture_name} case {case_index}: {field_name} {} is implausibly deep for {:?} \
             (bound {bound}, twice the smaller link's own bounding radius)",
            minimum_distance.distance,
            minimum_distance.link_names,
        );
    }
}

fn assert_full_parity_matches_oracle(model: &RobotModel, fixture_name: &str, srdf_file: &str) {
    let env = floor_env();
    let acm = build_acm(srdf_file);
    let fixture = load_fixture(fixture_name);
    for (case_index, case) in fixture.cases.iter().enumerate() {
        let mut state = build_state(model, &case.joint_values);
        let posed = state.update();

        let self_result =
            env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
        assert_eq!(
            self_result.collision, case.self_collision,
            "{fixture_name} case {case_index}: self_collision"
        );
        let robot_result =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
        assert_eq!(
            robot_result.collision, case.robot_collision,
            "{fixture_name} case {case_index}: robot_collision"
        );

        let distance_request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        // Penetration depth for two overlapping general (non-convex) meshes
        // is only asserted for its sign, not its magnitude -- see the module
        // doc's "no exact penetration-depth parity for interpenetrating
        // meshes" section. Separating distances (both sides report
        // `collision: false`) are architecturally comparable and asserted at
        // full tolerance.
        let self_distance = env.distance_self(&distance_request, &posed, &[]);
        if case.self_collision {
            assert!(
                self_distance.minimum_distance.distance <= TOLERANCE,
                "{fixture_name} case {case_index}: self_distance {} should be <= 0 \
                 (oracle reports self_collision)",
                self_distance.minimum_distance.distance
            );
            assert_plausible_depth(
                model,
                fixture_name,
                case_index,
                "self_distance",
                &self_distance.minimum_distance,
            );
        } else {
            assert!(
                (self_distance.minimum_distance.distance - case.self_distance).abs() < TOLERANCE,
                "{fixture_name} case {case_index}: self_distance {} != {} (oracle)",
                self_distance.minimum_distance.distance,
                case.self_distance
            );
        }
        let robot_distance = env.distance_robot(&distance_request, &posed, &[]);
        if case.robot_collision {
            assert!(
                robot_distance.minimum_distance.distance <= TOLERANCE,
                "{fixture_name} case {case_index}: robot_distance {} should be <= 0 \
                 (oracle reports robot_collision)",
                robot_distance.minimum_distance.distance
            );
            assert_plausible_depth(
                model,
                fixture_name,
                case_index,
                "robot_distance",
                &robot_distance.minimum_distance,
            );
        } else {
            assert!(
                (robot_distance.minimum_distance.distance - case.robot_distance).abs() < TOLERANCE,
                "{fixture_name} case {case_index}: robot_distance {} != {} (oracle)",
                robot_distance.minimum_distance.distance,
                case.robot_distance
            );
        }
    }
}

#[test]
fn panda_collision_matches_the_oracle() {
    let model = build_model("panda.urdf", "panda.srdf");
    assert_full_parity_matches_oracle(&model, "panda_collision.json", "panda.srdf");
}

#[test]
fn fanuc_collision_matches_the_oracle() {
    let model = build_model("fanuc.urdf", "fanuc.srdf");
    assert_full_parity_matches_oracle(&model, "fanuc_collision.json", "fanuc.srdf");
}

#[test]
fn pr2_collision_matches_the_oracle() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    assert_full_parity_matches_oracle(&model, "pr2_collision.json", "pr2.srdf");
}

/// Round 7 item 1: `parry.rs`'s deviation 6 blames every panda
/// `robot_distance` disagreement on this backend taking a single contact
/// where FCL's `distanceCallback` re-collides (up to 200 contacts) and keeps
/// the deepest. That explanation is testable, and for the sweep's own worst
/// case it is wrong.
///
/// `parry3d_f64::query::contact` between a mesh and a primitive does not
/// stop at one triangle: it dispatches to `contact_composite_shape_shape`
/// (`parry3d-f64-0.30.0/src/query/contact/contact_composite_shape_shape.rs`),
/// which visits every triangle whose AABB overlaps the other shape's and
/// keeps whichever `dist` is smallest -- i.e. deepest -- across all of them.
/// One `query::contact` call against a mesh is *already* a
/// maximum-over-the-contact-set reduction; there is no unexploited
/// aggregation left for this backend to add.
///
/// The seed-1, 10,000-case panda sweep's single worst disagreement (case
/// 3289) is concrete evidence this reduction is not the gap: oracle
/// `robot_distance = -2.80638374720525752` against this backend's
/// `-0.06879644150682307`, both naming the same closest pair
/// (`panda_link0`, `floor`). `panda_link0`'s own collision mesh spans at
/// most `bounding_radius` from its own local origin (asserted below,
/// computed from the real STL vertices this test loads through the same
/// [`RobotModel`] every other test in this file uses) -- nowhere near large
/// enough for a rigid body's own penetration depth to reach 2.8: a body
/// cannot overlap anything by more than its own diameter, in any
/// orientation, at any pose. No accumulation over `panda_link0`'s actual
/// triangles, however exhaustive, can produce oracle's -2.806; this
/// backend's -0.069 is the geometrically consistent answer for this pair.
///
/// So oracle's -2.806 is not a deeper true contact this backend fails to
/// find, and deviation 6's stated cause (single contact vs FCL's
/// 200-contact maximum) does not explain this disagreement. The oracle's
/// own FCL/libccd penetration-depth (EPA) computation is producing a
/// geometrically-impossible result for this arbitrarily-rotated, deeply
/// interpenetrating configuration -- a known libccd failure mode under deep
/// penetration, not a gap in this port. Whether that also explains every one
/// of the sweep's other ~9,500 distance-only disagreements is not claimed
/// here (each would need its own bounding-radius check); this test
/// falsifies deviation 6 as *this* case's cause, which is what "the worst
/// deviation" was cited for.
#[test]
fn panda_worst_sweep_deviation_is_not_a_missed_deeper_contact() {
    let model = build_model("panda.urdf", "panda.srdf");
    let acm = build_acm("panda.srdf");
    let env = floor_env();

    // Case 3289 of the seed-1, 10,000-case `tools/moveit-diff --collision`
    // sweep against panda: the sweep's single worst `robot_distance`
    // deviation (|d| = 2.738). Re-derived, not guessed: the oracle's
    // `random_states` RNG draws states strictly in sequence, so requesting
    // `count: 3290` with the same `seed: 1` reproduces states 0..=3289
    // identically to the original 10,000-case run, and `states[3289]` was
    // read directly off that response.
    let joint_values: BTreeMap<String, f64> = [
        ("panda_finger_joint1", 0.020648300275206567),
        ("panda_finger_joint2", 0.020648300275206567),
        ("panda_joint1", -2.1341387270914858),
        ("panda_joint2", 0.5692316297013313),
        ("panda_joint3", 2.775629999012407),
        ("panda_joint4", -1.1947285405220465),
        ("panda_joint5", -1.2969677427162882),
        ("panda_joint6", 2.576966588832438),
        ("panda_joint7", -2.012807961705187),
        ("virtual_joint/rot_w", 0.4993027569946919),
        ("virtual_joint/rot_x", 0.6067748702274111),
        ("virtual_joint/rot_y", -0.09358784320449841),
        ("virtual_joint/rot_z", -0.6113610466183941),
        ("virtual_joint/trans_x", 0.0),
        ("virtual_joint/trans_y", 0.0),
        ("virtual_joint/trans_z", 0.0),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_owned(), v))
    .collect();

    // The oracle's `collision` op answer for exactly this state, against
    // this same floor object -- read from the sweep log, not recomputed.
    const ORACLE_ROBOT_DISTANCE: f64 = -2.806_383_747_205_257_5;

    let mut state = build_state(&model, &joint_values);
    let posed = state.update();
    let request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };
    let robot_distance = env.distance_robot(&request, &posed, &[]);
    assert_eq!(
        robot_distance.minimum_distance.link_names,
        ["panda_link0".to_owned(), "floor".to_owned()],
        "expected panda_link0 to remain the closest pair for this case"
    );
    assert!(
        (robot_distance.minimum_distance.distance - (-0.06879644150682307)).abs() < 1e-9,
        "this backend's own robot_distance moved from the value this test's finding is about: {}",
        robot_distance.minimum_distance.distance
    );

    // panda_link0's own collision mesh's bounding radius from its own local
    // origin -- an upper bound on the penetration depth any accumulation
    // over its real triangles could ever report, since a rigid body cannot
    // overlap anything by more than its own diameter.
    let link0 = model
        .link_model("panda_link0")
        .expect("panda has panda_link0");
    let bounding_radius = link0
        .shapes()
        .iter()
        .filter_map(|shape| match &shape.shape {
            Shape::Mesh(mesh) => mesh
                .vertices
                .iter()
                .map(|v| {
                    (shape.origin_transform * nalgebra::Point3::from(*v))
                        .coords
                        .norm()
                })
                .fold(None::<f64>, |acc, d| Some(acc.map_or(d, |a| a.max(d)))),
            _ => None,
        })
        .fold(0.0_f64, f64::max);

    assert!(
        (0.0..1.0).contains(&bounding_radius),
        "panda_link0's mesh bounding radius {bounding_radius} was expected to be a small, \
         positive, sub-meter value"
    );
    assert!(
        ORACLE_ROBOT_DISTANCE.abs() > 2.0 * bounding_radius,
        "oracle's robot_distance {ORACLE_ROBOT_DISTANCE} does not exceed twice panda_link0's own \
         bounding radius {bounding_radius} -- this test's premise (that value is geometrically \
         impossible for this pair) no longer holds"
    );
}

/// Round-8 §38: case 7552 of the same seed-1, 10,000-case pr2 sweep the
/// module doc's "confirmed at three scales" paragraph cites -- the case that
/// motivated adding the oracle's `self_distance_pair`/`robot_distance_pair`
/// (`tools/moveit-oracle/src/oracle.cpp`'s `distancePairToJson`) in the first
/// place, because before that addition there was no way to name which pair
/// either backend's reported distance was even about.
///
/// The obvious hypothesis -- "the two sides agree on which pair is closest
/// and disagree only on how deep" -- is not what this case shows once the
/// pair is actually named. Both `self_distance` and `robot_distance` here
/// have *several* simultaneously-interpenetrating candidate pairs (see
/// `self_contacts`/`robot_contacts` in a live `collision` op response for
/// this state), each with its own independently-computed depth, and this
/// backend and the oracle each independently rank a *different* pair as the
/// global deepest:
///
/// - self: oracle ranks (`l_gripper_r_finger_link`, `l_gripper_palm_link`)
///   deepest at `-0.03027`; this backend ranks (`base_bellow_link`,
///   `torso_lift_link`) deepest at `-0.05293`. Isolating the oracle's own
///   pair (via an ACM that skips every other pair) gets this backend's
///   answer for *that specific pair*: `-0.00188` -- nowhere near its own
///   `-0.05293` worst, which is why it does not win here even though the
///   oracle picked it.
/// - robot: oracle ranks (`r_gripper_l_finger_link`, `floor`) deepest at
///   `-0.02488`; this backend ranks (`r_gripper_l_finger_tip_link`, `floor`)
///   deepest at `-0.05094`. Isolating the oracle's own pair gets `-0.02833`
///   here -- close to the oracle's `-0.02488` for that same pair (`|d| =
///   0.0035`) -- so the robot-side disagreement is almost entirely about
///   *which finger link* wins, not about the winning depth.
///
/// Both pairs in both queries are mesh-involving (`torso_lift_link`,
/// `r_gripper_l_finger_tip_link`, and the oracle's own picks, are all
/// meshes; see the `collision` op's `shape_kinds_1`/`shape_kinds_2`), so this
/// is deviation 6's mechanism (independent EPA implementations settling on
/// different local penetrations for an overlapping mesh pair), not a
/// different cause: when several such disagreements are close in magnitude,
/// a small per-pair difference is enough to flip which pair either side
/// reports as globally deepest. That is distinct from the visibility_cone
/// `max_contacts: 1` traversal-order tie-break (round-8 §38.3): that one
/// picks a first-found pair out of a *truncated, non-exhaustive* contact
/// query; `distance_self`/`distance_robot` never truncate (deviation 7), so
/// every pair here really was compared and the ranking is a real, if
/// unstable, disagreement about depth -- not an artifact of which pair a
/// budget-limited search happened to visit first.
///
/// None of these numbers are implausible the way panda's worst case was:
/// the bounding-radius check below confirms both backends' answers for the
/// robot-side pairs sit comfortably inside the relevant link's own bounding
/// radius, unlike panda's geometrically-impossible `-2.806`.
#[test]
fn pr2_case_7552_depth_disagreement_ranks_a_different_pair() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let acm = build_acm("pr2.srdf");
    let env = floor_env();

    // Case 7552 of the seed-1, 10,000-case `tools/moveit-diff --collision`
    // sweep against pr2. Re-derived, not guessed: the oracle's
    // `random_states` RNG draws states strictly in sequence, so requesting
    // `count: 7553` with the same `seed: 1` reproduces states 0..=7552
    // identically to the original 10,000-case run, and `states[7552]` was
    // read directly off that response.
    let joint_values: BTreeMap<String, f64> = [
        ("bl_caster_l_wheel_joint", -2.7530578545158697),
        ("bl_caster_r_wheel_joint", -0.6541500350832292),
        ("bl_caster_rotation_joint", -2.1071702944321746),
        ("br_caster_l_wheel_joint", -0.25144364721432044),
        ("br_caster_r_wheel_joint", 0.5714552888465483),
        ("br_caster_rotation_joint", 1.3459924924128535),
        ("fl_caster_l_wheel_joint", 1.6282587322204458),
        ("fl_caster_r_wheel_joint", -2.9044549042163976),
        ("fl_caster_rotation_joint", -0.028935971013624773),
        ("fr_caster_l_wheel_joint", 2.1960366795023853),
        ("fr_caster_r_wheel_joint", 1.4276562617623512),
        ("fr_caster_rotation_joint", -2.103756317620623),
        ("head_pan_joint", -2.7273416063464246),
        ("head_tilt_joint", 0.5973734346006577),
        ("l_elbow_flex_joint", -0.700751763412077),
        ("l_forearm_roll_joint", -0.9278961829433645),
        ("l_gripper_joint", 0.03171822756156325),
        ("l_gripper_l_finger_joint", 0.3815533448671922),
        ("l_gripper_l_finger_tip_joint", 0.3815533448671922),
        ("l_gripper_motor_screw_joint", -1.999628958401011),
        ("l_gripper_motor_slider_joint", 0.022971630422398442),
        ("l_gripper_r_finger_joint", 0.3815533448671922),
        ("l_gripper_r_finger_tip_joint", 0.3815533448671922),
        ("l_shoulder_lift_joint", 1.1877857735458996),
        ("l_shoulder_pan_joint", 0.4511944509654834),
        ("l_upper_arm_roll_joint", 1.2894796210341157),
        ("l_wrist_flex_joint", -0.11099946156609808),
        ("l_wrist_roll_joint", -3.134788791301619),
        ("laser_tilt_mount_joint", 0.5776462211810098),
        ("r_elbow_flex_joint", -0.4505700076777255),
        ("r_forearm_roll_joint", 1.7475075968314533),
        ("r_gripper_joint", 0.0027053921837359666),
        ("r_gripper_l_finger_joint", 0.14602129764202984),
        ("r_gripper_l_finger_tip_joint", 0.14602129764202984),
        ("r_gripper_motor_screw_joint", 2.4001195725111915),
        ("r_gripper_motor_slider_joint", -0.027126504015177494),
        ("r_gripper_r_finger_joint", 0.14602129764202984),
        ("r_gripper_r_finger_tip_joint", 0.14602129764202984),
        ("r_shoulder_lift_joint", 1.0406979827467353),
        ("r_shoulder_pan_joint", -0.06264072843631086),
        ("r_upper_arm_roll_joint", -3.3010135852731763),
        ("r_wrist_flex_joint", -0.15365014139097188),
        ("r_wrist_roll_joint", 0.07053784872167457),
        ("torso_lift_joint", 0.017635104052722454),
        ("torso_lift_motor_screw_joint", -2.209226948701471),
        ("world_joint/theta", 1.9908529625827018),
        ("world_joint/x", 0.0),
        ("world_joint/y", 0.0),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_owned(), v))
    .collect();

    // The oracle's `collision` op answer for exactly this state, against
    // this same floor object -- read from the sweep log and the oracle's
    // `self_distance_pair`/`robot_distance_pair` fields, not recomputed.
    const ORACLE_SELF_DISTANCE: f64 = -3.027_029_289_561_778e-2;
    const ORACLE_SELF_PAIR_RUST_DISTANCE: f64 = -1.880_864_788_888_060_7e-3;
    const ORACLE_ROBOT_DISTANCE: f64 = -2.487_585_155_291_470_5e-2;
    const ORACLE_ROBOT_PAIR_RUST_DISTANCE: f64 = -2.832_776_408_849_211_5e-2;

    let mut state = build_state(&model, &joint_values);
    let posed = state.update();
    let request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };

    // Skip every pair by default, then explicitly force just one back in --
    // isolates a single named pair's own distance out of the exhaustive
    // (deviation 7) minimum-distance search.
    let isolate_self_pair = |a: &str, b: &str| -> AllowedCollisionMatrix {
        let mut isolated = AllowedCollisionMatrix::new();
        for link in model.link_models() {
            isolated.set_default_entry(link.name(), true);
        }
        isolated.set_entry(a, b, false);
        isolated
    };
    let isolate_robot_pair = |target: &str| -> AllowedCollisionMatrix {
        let mut isolated = AllowedCollisionMatrix::new();
        for link in model.link_models() {
            if link.name() != target {
                isolated.set_default_entry(link.name(), true);
            }
        }
        isolated
    };

    let self_distance = env.distance_self(&request, &posed, &[]);
    assert_eq!(
        self_distance.minimum_distance.link_names,
        ["base_bellow_link".to_owned(), "torso_lift_link".to_owned()],
        "expected this backend's own worst self-collision pair for this case"
    );
    assert!(
        (self_distance.minimum_distance.distance - ORACLE_SELF_DISTANCE).abs() > 1e-3,
        "self_distance {} no longer disagrees with the oracle {ORACLE_SELF_DISTANCE} -- this \
         test's premise (a confirmed, still-open disagreement) no longer holds",
        self_distance.minimum_distance.distance
    );
    let oracle_self_pair_acm = isolate_self_pair("l_gripper_r_finger_link", "l_gripper_palm_link");
    let oracle_self_pair_req = DistanceRequest {
        acm: Some(&oracle_self_pair_acm),
        ..request
    };
    let oracle_self_pair_distance = env.distance_self(&oracle_self_pair_req, &posed, &[]);
    assert!(
        (oracle_self_pair_distance.minimum_distance.distance - ORACLE_SELF_PAIR_RUST_DISTANCE)
            .abs()
            < 1e-9,
        "this backend's own answer for the oracle's self pair moved from {} to {}",
        ORACLE_SELF_PAIR_RUST_DISTANCE,
        oracle_self_pair_distance.minimum_distance.distance
    );
    assert!(
        oracle_self_pair_distance.minimum_distance.distance
            > self_distance.minimum_distance.distance,
        "expected the oracle's own self pair ({}) to be shallower than this backend's own worst \
         self pair ({}) -- that gap is *why* this backend does not rank the oracle's pair first",
        oracle_self_pair_distance.minimum_distance.distance,
        self_distance.minimum_distance.distance
    );

    let robot_distance = env.distance_robot(&request, &posed, &[]);
    assert_eq!(
        robot_distance.minimum_distance.link_names,
        ["r_gripper_l_finger_tip_link".to_owned(), "floor".to_owned()],
        "expected this backend's own worst robot-collision pair for this case"
    );
    assert!(
        (robot_distance.minimum_distance.distance - ORACLE_ROBOT_DISTANCE).abs() > 1e-3,
        "robot_distance {} no longer disagrees with the oracle {ORACLE_ROBOT_DISTANCE} -- this \
         test's premise (a confirmed, still-open disagreement) no longer holds",
        robot_distance.minimum_distance.distance
    );
    let oracle_robot_pair_acm = isolate_robot_pair("r_gripper_l_finger_link");
    let oracle_robot_pair_req = DistanceRequest {
        acm: Some(&oracle_robot_pair_acm),
        ..request
    };
    let oracle_robot_pair_distance = env.distance_robot(&oracle_robot_pair_req, &posed, &[]);
    assert!(
        (oracle_robot_pair_distance.minimum_distance.distance - ORACLE_ROBOT_PAIR_RUST_DISTANCE)
            .abs()
            < 1e-9,
        "this backend's own answer for the oracle's robot pair moved from {} to {}",
        ORACLE_ROBOT_PAIR_RUST_DISTANCE,
        oracle_robot_pair_distance.minimum_distance.distance
    );
    assert!(
        (oracle_robot_pair_distance.minimum_distance.distance - ORACLE_ROBOT_DISTANCE).abs() < 5e-3,
        "expected this backend's answer for the oracle's own robot pair ({}) to sit close to the \
         oracle's ({ORACLE_ROBOT_DISTANCE}) -- unlike the self-side pair, the robot-side \
         disagreement is mostly about which pair wins, not the winning depth",
        oracle_robot_pair_distance.minimum_distance.distance
    );

    // Bounding radius from each pair's own local origin, the same
    // impossibility yardstick `panda_worst_sweep_deviation_is_not_a_missed_
    // deeper_contact` uses -- here to show the *opposite*: both sides'
    // numbers are comfortably plausible, not that either is impossible.
    let bounding_radius = |link_name: &str| -> f64 {
        let link = model
            .link_model(link_name)
            .unwrap_or_else(|e| panic!("pr2 has {link_name}: {e}"));
        link.shapes()
            .iter()
            .filter_map(|shape| match &shape.shape {
                Shape::Mesh(mesh) => mesh
                    .vertices
                    .iter()
                    .map(|v| {
                        (shape.origin_transform * nalgebra::Point3::from(*v))
                            .coords
                            .norm()
                    })
                    .fold(None::<f64>, |acc, d| Some(acc.map_or(d, |a| a.max(d)))),
                _ => None,
            })
            .fold(0.0_f64, f64::max)
    };
    // Twice the bounding radius, matching
    // `panda_worst_sweep_deviation_is_not_a_missed_deeper_contact`'s own
    // yardstick: a rigid body cannot overlap anything by more than its own
    // diameter, in any orientation, at any pose.
    let r_gripper_l_finger_tip_radius = bounding_radius("r_gripper_l_finger_tip_link");
    assert!(
        ORACLE_ROBOT_DISTANCE.abs() < 2.0 * r_gripper_l_finger_tip_radius
            && robot_distance.minimum_distance.distance.abs() < 2.0 * r_gripper_l_finger_tip_radius,
        "expected both robot_distance numbers ({}, {ORACLE_ROBOT_DISTANCE}) to sit inside twice \
         r_gripper_l_finger_tip_link's own bounding radius {r_gripper_l_finger_tip_radius} -- \
         unlike panda's worst case, neither should be a physically impossible penetration depth",
        robot_distance.minimum_distance.distance
    );
}

/// Round 9 item 1: `parry.rs`'s deviation 6 argument for case 7552's self
/// pair (`l_gripper_r_finger_link`/`l_gripper_palm_link`, this backend
/// `-0.00188` against the oracle's `-0.03027`) is that both are genuine,
/// exhaustive EPA answers for a general mesh-mesh overlap that are simply
/// not required to converge -- not this backend truncating its search under
/// a tight prediction margin. That is testable directly: if the answer
/// changed with the prediction margin, a narrowed search (not an inherent
/// EPA disagreement) would be the more likely explanation.
///
/// It does not change. `query::contact` at prediction `0.0` (the tightest
/// possible: touching-or-penetrating only), `0.1`, and `1.0` all return the
/// identical contact for this pair at case 7552's exact state -- ruling out
/// prediction-margin truncation as a factor for this pair specifically, the
/// same way `accumulate_distance`'s `Global`-threshold narrowing was ruled
/// out for `base_bellow_link`/`torso_lift_link` below by using an isolated
/// ACM (no competing pair to narrow the threshold against in the first
/// place).
#[test]
fn gripper_pair_contact_is_prediction_invariant() {
    use parry3d_f64::query;
    use parry3d_f64::shape::TriMesh;

    let model = build_model("pr2.urdf", "pr2.srdf");
    // Case 7552 of the seed-1, 10,000-case pr2 sweep -- the same state
    // `pr2_case_7552_depth_disagreement_ranks_a_different_pair` uses.
    let joint_values: BTreeMap<String, f64> = [
        ("bl_caster_l_wheel_joint", -2.7530578545158697),
        ("bl_caster_r_wheel_joint", -0.6541500350832292),
        ("bl_caster_rotation_joint", -2.1071702944321746),
        ("br_caster_l_wheel_joint", -0.25144364721432044),
        ("br_caster_r_wheel_joint", 0.5714552888465483),
        ("br_caster_rotation_joint", 1.3459924924128535),
        ("fl_caster_l_wheel_joint", 1.6282587322204458),
        ("fl_caster_r_wheel_joint", -2.9044549042163976),
        ("fl_caster_rotation_joint", -0.028935971013624773),
        ("fr_caster_l_wheel_joint", 2.1960366795023853),
        ("fr_caster_r_wheel_joint", 1.4276562617623512),
        ("fr_caster_rotation_joint", -2.103756317620623),
        ("head_pan_joint", -2.7273416063464246),
        ("head_tilt_joint", 0.5973734346006577),
        ("l_elbow_flex_joint", -0.700751763412077),
        ("l_forearm_roll_joint", -0.9278961829433645),
        ("l_gripper_joint", 0.03171822756156325),
        ("l_gripper_l_finger_joint", 0.3815533448671922),
        ("l_gripper_l_finger_tip_joint", 0.3815533448671922),
        ("l_gripper_motor_screw_joint", -1.999628958401011),
        ("l_gripper_motor_slider_joint", 0.022971630422398442),
        ("l_gripper_r_finger_joint", 0.3815533448671922),
        ("l_gripper_r_finger_tip_joint", 0.3815533448671922),
        ("l_shoulder_lift_joint", 1.1877857735458996),
        ("l_shoulder_pan_joint", 0.4511944509654834),
        ("l_upper_arm_roll_joint", 1.2894796210341157),
        ("l_wrist_flex_joint", -0.11099946156609808),
        ("l_wrist_roll_joint", -3.134788791301619),
        ("laser_tilt_mount_joint", 0.5776462211810098),
        ("r_elbow_flex_joint", -0.4505700076777255),
        ("r_forearm_roll_joint", 1.7475075968314533),
        ("r_gripper_joint", 0.0027053921837359666),
        ("r_gripper_l_finger_joint", 0.14602129764202984),
        ("r_gripper_l_finger_tip_joint", 0.14602129764202984),
        ("r_gripper_motor_screw_joint", 2.4001195725111915),
        ("r_gripper_motor_slider_joint", -0.027126504015177494),
        ("r_gripper_r_finger_joint", 0.14602129764202984),
        ("r_gripper_r_finger_tip_joint", 0.14602129764202984),
        ("r_shoulder_lift_joint", 1.0406979827467353),
        ("r_shoulder_pan_joint", -0.06264072843631086),
        ("r_upper_arm_roll_joint", -3.3010135852731763),
        ("r_wrist_flex_joint", -0.15365014139097188),
        ("r_wrist_roll_joint", 0.07053784872167457),
        ("torso_lift_joint", 0.017635104052722454),
        ("torso_lift_motor_screw_joint", -2.209226948701471),
        ("world_joint/theta", 1.9908529625827018),
        ("world_joint/x", 0.0),
        ("world_joint/y", 0.0),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_owned(), v))
    .collect();

    let mut state = build_state(&model, &joint_values);
    let posed = state.update();

    let mesh_shape = |name: &str| -> (Isometry3, TriMesh) {
        let link = model
            .link_model(name)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let global = posed.global_link_transform(name).unwrap();
        let ls = &link.shapes()[0];
        let Shape::Mesh(m) = &ls.shape else {
            panic!("{name} shape[0] is not a mesh");
        };
        let verts = m
            .vertices
            .iter()
            .map(|v| parry3d_f64::math::Vector::new(v.x, v.y, v.z))
            .collect();
        let mesh = TriMesh::new(verts, m.triangles.clone()).expect("valid trimesh");
        (global * ls.origin_transform, mesh)
    };

    let (pose_a, mesh_a) = mesh_shape("l_gripper_r_finger_link");
    let (pose_b, mesh_b) = mesh_shape("l_gripper_palm_link");
    let pose_a_parry: parry3d_f64::math::Pose = pose_a.into();
    let pose_b_parry: parry3d_f64::math::Pose = pose_b.into();

    // The already-committed answer for this pair
    // (`pr2_case_7552_depth_disagreement_ranks_a_different_pair`'s
    // `ORACLE_SELF_PAIR_RUST_DISTANCE`).
    const EXPECTED_DIST: f64 = -1.880_864_788_888_060_7e-3;
    for prediction in [0.0_f64, 0.1, 1.0] {
        let contact = query::contact(&pose_a_parry, &mesh_a, &pose_b_parry, &mesh_b, prediction)
            .unwrap_or_else(|e| panic!("query::contact at prediction {prediction}: {e}"))
            .unwrap_or_else(|| panic!("expected a contact at prediction {prediction}"));
        assert!(
            (contact.dist - EXPECTED_DIST).abs() < 1e-9,
            "prediction {prediction}: dist {} != {EXPECTED_DIST} -- prediction margin changed \
             this pair's answer, contradicting this test's premise",
            contact.dist
        );
    }
}

/// Round 9 items 2 and 43, revised by round 10 §56/§56.3:
/// `base_bellow_link`/`torso_lift_link` (a `<box>` against a `<mesh>`, not
/// the mesh-mesh pairs deviation 6's other confirmed instances involve) is
/// this backend's own dominant self-collision constant across the seed-1,
/// 10,000-case pr2 sweep, and the one pair on all of pr2 whose relative pose
/// is a function of a *single* joint: `base_bellow_link` is fixed to
/// `base_link`, `torso_lift_link` moves along `torso_lift_joint` alone
/// (range `[0.0, 0.33]`), so every other one of pr2's ~46 other joints is
/// irrelevant to this specific pair's own distance -- its whole plateau/ramp
/// shape over that one joint's range is directly inspectable, not just
/// spot-checked at one state.
///
/// Round 9 argued that because the same local box face and the same local
/// mesh feature are found at two states inside the plateau, "the true
/// penetration depth genuinely is constant there," and therefore it must be
/// the oracle's own varying answer that is wrong. **That the true depth
/// cannot change over that span is the thing in dispute; assuming it is how
/// the argument reached its conclusion.** The oracle's own sweep across
/// `torso_lift_joint = 0.00` to `0.30` (step `0.02`) is not erratic: from
/// `0.12` to `0.22` it decreases smoothly and monotonically, shallowing at
/// very nearly `1:1` with the joint travel -- the signature of a genuine
/// z-direction overlap, not EPA noise. "Fails to hold constant" does not
/// survive that.
///
/// What does survive: this backend's own curve is
/// `min(candidate_x, candidate_z(t))`. `candidate_x` is the `-x`-face-vs-mesh
/// planar contact this test's plateau samples establish as
/// `torso_lift_joint`-invariant (neither body's local geometry in that
/// direction depends on the joint). `candidate_z(t)` is a genuinely
/// `t`-dependent z-direction separating-distance candidate -- this test's
/// ramp samples confirm it is linear in `t` with slope `1` (a rigid
/// z-translation of the mesh shifts a z-direction separating distance by
/// exactly the translation, a geometric identity, not a curve fit).
/// Penetration depth *is* the minimum translation that separates; when two
/// directions both separate, the answer is the shallower one. Below the
/// crossover, `candidate_x` is shallower and this backend reports the
/// plateau; past it, `candidate_z(t)` shallows further and wins, and this
/// backend's answer should then land on the oracle's own -- not merely
/// resemble it. That is the falsifiable claim this test asserts, not just
/// describes: it bisects this backend's own *observed* crossover
/// independently of the two fitted lines, confirms the two candidates
/// actually meet there to within this crate's own [`TOLERANCE`] (confirming
/// `min(...)` as the mechanism, rather than inferring it from two agreeing
/// samples), and checks agreement with a real captured oracle response
/// (`pr2_torso_lift_bellow_sweep_{request,response}.json`, read via
/// [`load_torso_sweep_oracle_points`]) at `torso_lift_joint = 0.22`, past the
/// crossover, to `~1e-9` -- the assertion that carries weight, since a test
/// that only checks two points inside the flat region could never
/// distinguish "correctly taking the minimum" from "stuck". See `parry.rs`'s
/// deviation 6 for the generalization from mesh-vs-mesh to
/// convex-primitive-vs-mesh this licenses, and for the two pair-families
/// that made this deviation look like three frozen constants at first.
///
/// Every sampled distance also passes through [`assert_plausible_depth`]
/// (reused, not reimplemented, from [`assert_full_parity_matches_oracle`]'s
/// own companion check): this is not a magnitude blow-up like panda's worst
/// case.
#[test]
fn pr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let env = floor_env();
    let mut acm = AllowedCollisionMatrix::new();
    for link in model.link_models() {
        acm.set_default_entry(link.name(), true);
    }
    acm.set_entry("base_bellow_link", "torso_lift_link", false);

    let bellow_torso_distance = |torso: f64| -> f64 {
        let mut joint_values = BTreeMap::new();
        joint_values.insert("torso_lift_joint".to_owned(), torso);
        let mut state = build_state(&model, &joint_values);
        let posed = state.update();
        let request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let distance = env.distance_self(&request, &posed, &[]);
        assert_eq!(
            distance.minimum_distance.link_names,
            ["base_bellow_link".to_owned(), "torso_lift_link".to_owned()],
            "torso_lift_joint={torso}: isolated ACM must report this pair"
        );
        assert_plausible_depth(
            &model,
            "pr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates",
            0,
            "self_distance",
            &distance.minimum_distance,
        );
        distance.minimum_distance.distance
    };

    // `candidate_x`: sampled at three points spread across the plateau
    // (round 9's own `EXPECTED_DIST`, at `torso_lift_joint = 0.1`, is one of
    // them). All three must agree -- the plateau is not just two
    // coincidentally-equal samples.
    let candidate_x_states = [0.02_f64, 0.10, 0.18];
    let candidate_x_samples = candidate_x_states.map(bellow_torso_distance);
    let candidate_x = candidate_x_samples[0];
    for (t, d) in candidate_x_states.iter().zip(candidate_x_samples) {
        assert!(
            (d - candidate_x).abs() < 1e-9,
            "torso_lift_joint={t}: candidate_x sample {d} != {candidate_x} -- the plateau is not \
             actually constant across these states"
        );
    }

    // `candidate_z`: sampled at three points spread across the ramp, well
    // past the `(0.20, 0.22)` bracket below. Fit as a line from the two
    // endpoints, then confirmed to actually *be* linear (not merely a
    // two-point secant) by checking the interior point against the fitted
    // line.
    let (t_z0, t_z1, t_z2) = (0.23_f64, 0.24, 0.25);
    let (d_z0, d_z1, d_z2) = (
        bellow_torso_distance(t_z0),
        bellow_torso_distance(t_z1),
        bellow_torso_distance(t_z2),
    );
    let candidate_z_slope = (d_z2 - d_z0) / (t_z2 - t_z0);
    assert!(
        (candidate_z_slope - 1.0).abs() < 1e-6,
        "candidate_z's fitted slope {candidate_z_slope} != 1.0 -- a rigid z-translation of the \
         mesh by dt should shift a z-direction separating distance by exactly dt"
    );
    let predicted_d_z1 = d_z0 + candidate_z_slope * (t_z1 - t_z0);
    assert!(
        (d_z1 - predicted_d_z1).abs() < 1e-9,
        "candidate_z at {t_z1} is {d_z1}, but the line through {t_z0}/{t_z2} predicts \
         {predicted_d_z1} -- candidate_z is not actually linear, only its two-point secant is"
    );
    let candidate_z = |t: f64| d_z0 + candidate_z_slope * (t - t_z0);

    // The predicted crossover: where the constant `candidate_x` line meets
    // the fitted `candidate_z` line.
    let predicted_crossover = t_z0 + (candidate_x - d_z0) / candidate_z_slope;
    const BRACKET: (f64, f64) = (0.20, 0.22);
    assert!(
        (BRACKET.0..=BRACKET.1).contains(&predicted_crossover),
        "predicted crossover {predicted_crossover} fell outside round 10's own bracket {BRACKET:?}"
    );

    // The *observed* crossover, bisected from this backend's own live
    // `distance_self` calls -- not the fitted lines -- so `min(...)` is
    // confirmed as the actual mechanism rather than inferred only from the
    // fit above.
    let still_on_plateau = |t: f64| (bellow_torso_distance(t) - candidate_x).abs() < 1e-9;
    assert!(
        still_on_plateau(BRACKET.0),
        "torso_lift_joint={}: expected this to still be on the plateau",
        BRACKET.0
    );
    assert!(
        !still_on_plateau(BRACKET.1),
        "torso_lift_joint={}: expected this to already be past the crossover",
        BRACKET.1
    );
    let (mut lo, mut hi) = BRACKET;
    for _ in 0..30 {
        let mid = 0.5 * (lo + hi);
        if still_on_plateau(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let observed_crossover = lo;
    assert!(
        (observed_crossover - predicted_crossover).abs() < 1e-4,
        "bisected crossover {observed_crossover} != predicted {predicted_crossover} -- min(...) \
         does not actually explain this backend's own curve"
    );

    // At the crossover, the two candidates must actually be equal -- not
    // merely both close to whatever this backend reports there -- to within
    // this crate's own [`TOLERANCE`] ("the backend's own tolerance", round
    // 10's own phrasing).
    assert!(
        (candidate_x - candidate_z(observed_crossover)).abs() < TOLERANCE,
        "at the crossover, candidate_x {candidate_x} and candidate_z {} differ by more than \
         {TOLERANCE} -- min(...) requires the two candidates to actually meet there",
        candidate_z(observed_crossover)
    );

    // The falsifiable claim itself, checked against a real captured oracle
    // response, not a transcribed literal: inside the plateau this backend
    // disagrees with the oracle (that disagreement is deviation 6, not a
    // hole in this port); past the crossover, `min(...)` predicts this
    // backend's own answer should match the oracle's, not merely resemble
    // it.
    for point in load_torso_sweep_oracle_points() {
        let rust_distance = bellow_torso_distance(point.torso_lift_joint);
        if point.torso_lift_joint < observed_crossover {
            assert!(
                (rust_distance - point.self_distance).abs() > 1e-3,
                "torso_lift_joint={}: expected this backend's plateau ({rust_distance}) to \
                 still disagree with the oracle ({}) here",
                point.torso_lift_joint,
                point.self_distance
            );
        } else {
            assert!(
                (rust_distance - point.self_distance).abs() < 1e-9,
                "torso_lift_joint={}: this backend {rust_distance} != oracle {} past the \
                 crossover -- the falsifiable claim this test exists to check",
                point.torso_lift_joint,
                point.self_distance
            );
        }
    }
}

/// One `robot_distance`/`robot_distance_pair` case from
/// `pr2_world_object_same_pair_{request,response}.json` -- the two states
/// (seed 20260804, `right_arm`, `--collision`, case indices 422 and 2996 of
/// the 3000-case sweep PORTING-PLAN.md §60 recorded) where p1-joints' sweep
/// and this port agree on *which* pair is the world-object argmin
/// (`l_gripper_{l,r}_finger_tip_link`/`floor`) but disagree on its
/// magnitude, assigning the disagreement to p3-acm since §56's
/// min-of-two-candidates ranking mechanism has nothing to say when both
/// sides already pick the same pair.
struct WorldSamePairOraclePoint {
    joint_values: BTreeMap<String, f64>,
    oracle_distance: f64,
    link_name: String,
}

fn load_world_same_pair_oracle_points() -> Vec<WorldSamePairOraclePoint> {
    #[derive(Deserialize)]
    struct RequestCase {
        id: u64,
        joint_values: BTreeMap<String, f64>,
    }
    #[derive(Deserialize)]
    struct DistancePair {
        body_name_1: String,
        body_name_2: String,
    }
    #[derive(Deserialize)]
    struct ResultCase {
        robot_distance: f64,
        robot_distance_pair: DistancePair,
    }
    #[derive(Deserialize)]
    struct ResponseCase {
        id: u64,
        result: ResultCase,
    }

    let requests: Vec<RequestCase> = {
        let path = fixture_path("pr2_world_object_same_pair_request.json");
        let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
    };
    let responses: Vec<ResponseCase> = {
        let path = fixture_path("pr2_world_object_same_pair_response.json");
        let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
    };
    let responses_by_id: BTreeMap<u64, &ResponseCase> =
        responses.iter().map(|r| (r.id, r)).collect();

    requests
        .iter()
        .map(|req| {
            let response = responses_by_id
                .get(&req.id)
                .unwrap_or_else(|| panic!("no response for request id {}", req.id));
            let pair = &response.result.robot_distance_pair;
            let (world_side, robot_side) = if pair.body_name_2 == "floor" {
                (pair.body_name_2.as_str(), pair.body_name_1.as_str())
            } else {
                (pair.body_name_1.as_str(), pair.body_name_2.as_str())
            };
            assert_eq!(
                world_side, "floor",
                "request id {}: fixture is meant to capture a robot-link/floor pair",
                req.id
            );
            WorldSamePairOraclePoint {
                joint_values: req.joint_values.clone(),
                oracle_distance: response.result.robot_distance,
                link_name: robot_side.to_owned(),
            }
        })
        .collect()
}

/// The most-negative global z among `link_name`'s own mesh vertices at
/// `posed`'s pose. `floor_env`'s box has its top face at `z = 0` and spans
/// `x, y in [-2, 2]` -- far larger than any pr2 fingertip mesh -- so a
/// vertex under that footprint at `z = -d` can only be moved clear of the
/// box by a translation of at least `d`, straight up: sideways requires
/// crossing to the footprint edge (>1.5m here) and down requires crossing
/// the box's full 0.1m thickness, both far more expensive. `-d` is
/// therefore an independent, non-collision-pipeline lower bound on the true
/// minimum translation distance for that vertex.
fn deepest_vertex_under_floor(
    model: &RobotModel,
    posed: &cspace_core::state::Posed<'_, '_>,
    link_name: &str,
) -> f64 {
    let link = model
        .link_model(link_name)
        .unwrap_or_else(|e| panic!("{link_name}: {e}"));
    let shape = &link.shapes()[0];
    let Shape::Mesh(mesh) = &shape.shape else {
        panic!("{link_name} shape[0] is not a mesh");
    };
    let global = posed
        .global_link_transform(link_name)
        .unwrap_or_else(|e| panic!("{link_name}: {e}"))
        * shape.origin_transform;
    let (min_z, min_x, min_y) =
        mesh.vertices
            .iter()
            .fold((f64::INFINITY, 0.0, 0.0), |(min_z, min_x, min_y), v| {
                let p = global.transform_point(&nalgebra::Point3::from(*v));
                if p.z < min_z {
                    (p.z, p.x, p.y)
                } else {
                    (min_z, min_x, min_y)
                }
            });
    assert!(
        min_x.abs() < 1.9 && min_y.abs() < 1.9,
        "{link_name}'s deepest vertex ({min_x}, {min_y}, {min_z}) is not safely inside floor_env's \
         4x4 footprint -- the straight-up-is-cheapest argument this bound relies on no longer holds"
    );
    -min_z
}

/// Round 11 item 1 (PORTING-PLAN.md §60): p1-joints' seed-20260804
/// `right_arm --collision` sweep found two cases where this backend and the
/// oracle already agree on the argmin world-object pair
/// (`l_gripper_{l,r}_finger_tip_link`/`floor`) but disagree on its depth --
/// this backend deeper both times (case 422: oracle -0.011274 vs this
/// backend -0.015686; case 2996: oracle -0.009943 vs this backend
/// -0.012375). Penetration depth is the *minimum* translation distance, so
/// a deeper answer is only legitimate if the shallower one does not
/// correspond to any real separating direction.
///
/// `deepest_vertex_under_floor` answers that independently of both
/// backends' collision pipelines: it is a raw mesh-vertex measurement, not
/// a re-run of either's distance query. For both cases it reproduces this
/// backend's own reported magnitude to `TOLERANCE`, and the vertex it finds
/// sits well inside the floor's 4x4 footprint (asserted above), where
/// straight up is the only cheap escape (see that function's doc). So the
/// oracle's shallower number does not correspond to an achievable
/// separating translation here -- `distanceRobot`'s own re-collide-and-
/// take-max-depth search (see `parry.rs`'s deviation-6 doc) still misses
/// this mesh's true deepest point. That is deviation-6's mechanism (FCL's
/// non-convex penetration depth is an approximation, not exact EPA) landing
/// on a *magnitude* disagreement for a pair both sides already rank as the
/// argmin, rather than the *pair-ranking* disagreement deviation-6's other
/// instances show -- which is exactly why `DistancePairStats`' pair-name
/// comparison (agreement on names) did not flag these two. Not a defect in
/// `parry.rs`: the deeper number is the one backed by a real mesh vertex.
#[test]
fn pr2_world_object_same_pair_deeper_depth_is_a_real_vertex_not_a_spurious_direction() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let acm = build_acm("pr2.srdf");
    let env = floor_env();

    for point in load_world_same_pair_oracle_points() {
        let mut state = build_state(&model, &point.joint_values);
        let posed = state.update();
        let request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let distance = env.distance_robot(&request, &posed, &[]);

        assert!(
            distance
                .minimum_distance
                .link_names
                .contains(&point.link_name)
                && distance
                    .minimum_distance
                    .link_names
                    .contains(&"floor".to_owned()),
            "{}: oracle's argmin pair is {}/floor, this backend's is {:?} -- expected agreement \
             on the pair, disagreement only on depth",
            point.link_name,
            point.link_name,
            distance.minimum_distance.link_names
        );
        assert!(
            distance.minimum_distance.distance < point.oracle_distance,
            "{}: this backend's {} is not deeper than oracle's {} -- the case this test exists \
             to characterize has flipped direction",
            point.link_name,
            distance.minimum_distance.distance,
            point.oracle_distance
        );

        let vertex_mtd = deepest_vertex_under_floor(&model, &posed, &point.link_name);
        assert!(
            (vertex_mtd - (-distance.minimum_distance.distance)).abs() < TOLERANCE,
            "{}: independent straight-up vertex bound {vertex_mtd} != this backend's own {} -- \
             this backend's deeper number no longer matches a real mesh vertex",
            point.link_name,
            -distance.minimum_distance.distance
        );
    }
}

/// Round 9's original find (§53.3) and §60.2's own listing: the seed-20260804
/// `right_arm --collision` 300-case sweep's case 122, one of the nine world-
/// object robot-distance cases whose deviation exceeds Phase 3's `1e-4` and
/// whose two sides pick *different* argmin pairs -- not one of
/// [`pr2_world_object_same_pair_deeper_depth_is_a_real_vertex_not_a_spurious_direction`]'s
/// two same-pair cases. Oracle: `l_gripper_r_finger_link`/`floor` at
/// `-1.17505058621331926e-2`. This port: `l_gripper_r_finger_tip_link`/
/// `floor` at `-3.30976249554740254e-2`, about 2.8x deeper.
///
/// PORTING-PLAN.md §56.4's third residual bullet read this as a case the
/// plateau/min-of-two-candidates explanation (§56.3) does not cover, "the
/// direction is reversed" -- and it is not covered by that specific formula,
/// which is derived for an unrelated pair's geometry. But
/// `deepest_vertex_under_floor` -- the same independent, both-backends-blind
/// mesh-vertex measurement the same-pair test above uses -- answers the
/// actual question: are *both* reported numbers real vertices, just of
/// different links?
///
/// MEASURED: `l_gripper_r_finger_tip_link`'s own deepest vertex under the
/// floor is `3.30976249554739491e-2`, matching this port's reported
/// magnitude to `TOLERANCE`. `l_gripper_r_finger_link`'s own deepest vertex
/// is `1.17505058621332897e-2`, matching the *oracle's* reported magnitude
/// to `TOLERANCE` -- not this port's. Both sides are reporting a real vertex
/// depth; they simply picked different links as the global argmin. Since
/// penetration severity is the deepest overlap, not the shallowest, the pair
/// that actually reaches deeper is the correct argmin, and this port found
/// it while the oracle did not. That is §302.4's already-documented mechanism
/// (`fcl-distance-threshold-suppresses-deeper-pairs`, upstream stopping its
/// per-pair search once a shallower penetrating pair is found) landing on a
/// world-object case, not a new one -- so the reversed direction this bullet
/// flagged is this port being *more* correct, not less.
#[test]
fn pr2_world_object_pair_flip_case_122_both_sides_are_real_vertices() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let acm = build_acm("pr2.srdf");
    let env = floor_env();

    // Case 122's own joint values -- as `tools/moveit-diff --pair-probe-json`
    // recorded them for that state (seed 20260804, `--group right_arm`,
    // `--cases 300`) -- reproduced live and confirmed identical to §53.3's
    // original oracle/port values to 10+ significant digits before this test
    // was written.
    let joints: &[(&str, f64)] = &[
        ("bl_caster_l_wheel_joint", 1.8110843327376944),
        ("bl_caster_r_wheel_joint", -1.3565553651303288),
        ("bl_caster_rotation_joint", -2.0946291775757473),
        ("br_caster_l_wheel_joint", -2.8638606040208527),
        ("br_caster_r_wheel_joint", -1.240851715711764),
        ("br_caster_rotation_joint", 0.7775410132584644),
        ("fl_caster_l_wheel_joint", -2.5131292699901304),
        ("fl_caster_r_wheel_joint", 2.428917916580673),
        ("fl_caster_rotation_joint", -1.157821668651315),
        ("fr_caster_l_wheel_joint", -1.8029447004321946),
        ("fr_caster_r_wheel_joint", 1.6507924616978693),
        ("fr_caster_rotation_joint", 0.34544718322748835),
        ("head_pan_joint", -0.16913603377528474),
        ("head_tilt_joint", 0.5262407144957595),
        ("l_elbow_flex_joint", -0.15970503807084158),
        ("l_forearm_roll_joint", 3.0863743147586256),
        ("l_gripper_joint", 0.071583628911525),
        ("l_gripper_l_finger_joint", 0.45979399859160186),
        ("l_gripper_l_finger_tip_joint", 0.45979399859160186),
        ("l_gripper_motor_screw_joint", 0.16252357158768227),
        ("l_gripper_motor_slider_joint", 0.05244462559930982),
        ("l_gripper_r_finger_joint", 0.45979399859160186),
        ("l_gripper_r_finger_tip_joint", 0.45979399859160186),
        ("l_shoulder_lift_joint", 1.1735535482576585),
        ("l_shoulder_pan_joint", 0.49178570006825517),
        ("l_upper_arm_roll_joint", -0.6450686627067626),
        ("l_wrist_flex_joint", -0.15731475260108718),
        ("l_wrist_roll_joint", -1.0238852147804027),
        ("laser_tilt_mount_joint", 1.38974086358116),
        ("r_elbow_flex_joint", -1.7188994917039528),
        ("r_forearm_roll_joint", 0.5885763984437711),
        ("r_gripper_joint", 0.06041045605018735),
        ("r_gripper_l_finger_joint", 0.02140877056401223),
        ("r_gripper_l_finger_tip_joint", 0.02140877056401223),
        ("r_gripper_motor_screw_joint", 3.1167337328588207),
        ("r_gripper_motor_slider_joint", -0.0832570452708751),
        ("r_gripper_r_finger_joint", 0.02140877056401223),
        ("r_gripper_r_finger_tip_joint", 0.02140877056401223),
        ("r_shoulder_lift_joint", 0.18841398464860393),
        ("r_shoulder_pan_joint", 0.43360092085106183),
        ("r_upper_arm_roll_joint", -2.3519548690877854),
        ("r_wrist_flex_joint", -1.5207342812791467),
        ("r_wrist_roll_joint", -0.148801931043419),
        ("torso_lift_joint", 0.01276570774964057),
        ("torso_lift_motor_screw_joint", -3.0057428418484173),
        ("world_joint/theta", -2.5874911909597635),
        ("world_joint/x", 0.0),
        ("world_joint/y", 0.0),
    ];
    let joint_values: BTreeMap<String, f64> = joints
        .iter()
        .map(|(name, value)| ((*name).to_owned(), *value))
        .collect();
    let mut state = build_state(&model, &joint_values);
    let posed = state.update();
    let request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };
    let distance = env.distance_robot(&request, &posed, &[]);
    assert_eq!(
        distance.minimum_distance.link_names,
        ["l_gripper_r_finger_tip_link".to_owned(), "floor".to_owned()],
        "case 122: this port's argmin pair changed -- no longer the pair-flip case this test \\
         exists to characterize"
    );

    let tip_vertex = deepest_vertex_under_floor(&model, &posed, "l_gripper_r_finger_tip_link");
    assert!(
        (tip_vertex - (-distance.minimum_distance.distance)).abs() < TOLERANCE,
        "l_gripper_r_finger_tip_link: independent straight-up vertex bound {tip_vertex} != \\
         this backend's own {}",
        -distance.minimum_distance.distance
    );

    // The oracle's own pick, checked against the *same* independent
    // instrument: its reported value should match a real vertex of the link
    // *it* named, not this port's.
    const ORACLE_LINK: &str = "l_gripper_r_finger_link";
    const ORACLE_DISTANCE: f64 = -1.175_050_586_213_319_3e-2;
    let oracle_link_vertex = deepest_vertex_under_floor(&model, &posed, ORACLE_LINK);
    assert!(
        (oracle_link_vertex - (-ORACLE_DISTANCE)).abs() < TOLERANCE,
        "{ORACLE_LINK}: independent straight-up vertex bound {oracle_link_vertex} != the \\
         oracle's own published {}",
        -ORACLE_DISTANCE
    );

    // The falsifiable claim: this port's pick is the deeper -- hence
    // correct -- global argmin, not an unexplained magnitude blow-up.
    assert!(
        tip_vertex > oracle_link_vertex,
        "l_gripper_r_finger_tip_link's own vertex depth {tip_vertex} is not deeper than \\
         l_gripper_r_finger_link's {oracle_link_vertex} -- the oracle's pick would then be the \\
         correct global argmin and this port's deeper answer would be unexplained"
    );
}

/// One `self_distance`/`self_distance_pair` case from
/// `pr2_self_wheel_same_pair_oracle_implausible_{request,response}.json` --
/// three states (seed 20260804, `right_arm --collision`, case indices 216,
/// 971 and 1265 of the same 3000-case sweep item 1's other fixtures use)
/// where this backend and the oracle agree on the self-side argmin pair
/// (`base_link`/one of the eight `*_caster_*_wheel_link`s, the same
/// rotationally-symmetric family `parry.rs`'s deviation-6 doc already names
/// as producing this backend's own frozen `-0.046592m` constant) but
/// disagree on magnitude far outside that pair's own bounding-radius bound.
struct SelfWheelOraclePoint {
    joint_values: BTreeMap<String, f64>,
    oracle_distance: f64,
    wheel_link_name: String,
}

fn load_self_wheel_oracle_points() -> Vec<SelfWheelOraclePoint> {
    #[derive(Deserialize)]
    struct RequestCase {
        id: u64,
        joint_values: BTreeMap<String, f64>,
    }
    #[derive(Deserialize)]
    struct DistancePair {
        body_name_1: String,
        body_name_2: String,
    }
    #[derive(Deserialize)]
    struct ResultCase {
        self_distance: f64,
        self_distance_pair: DistancePair,
    }
    #[derive(Deserialize)]
    struct ResponseCase {
        id: u64,
        result: ResultCase,
    }

    let requests: Vec<RequestCase> = {
        let path = fixture_path("pr2_self_wheel_same_pair_oracle_implausible_request.json");
        let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
    };
    let responses: Vec<ResponseCase> = {
        let path = fixture_path("pr2_self_wheel_same_pair_oracle_implausible_response.json");
        let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
    };
    let responses_by_id: BTreeMap<u64, &ResponseCase> =
        responses.iter().map(|r| (r.id, r)).collect();

    requests
        .iter()
        .map(|req| {
            let response = responses_by_id
                .get(&req.id)
                .unwrap_or_else(|| panic!("no response for request id {}", req.id));
            let pair = &response.result.self_distance_pair;
            let (base_side, wheel_side) = if pair.body_name_1 == "base_link" {
                (pair.body_name_1.as_str(), pair.body_name_2.as_str())
            } else {
                (pair.body_name_2.as_str(), pair.body_name_1.as_str())
            };
            assert_eq!(
                base_side, "base_link",
                "request id {}: fixture is meant to capture a base_link/wheel pair",
                req.id
            );
            assert!(
                wheel_side.ends_with("_caster_l_wheel_link")
                    || wheel_side.ends_with("_caster_r_wheel_link"),
                "request id {}: {wheel_side} is not a caster wheel link",
                req.id
            );
            SelfWheelOraclePoint {
                joint_values: req.joint_values.clone(),
                oracle_distance: response.result.self_distance,
                wheel_link_name: wheel_side.to_owned(),
            }
        })
        .collect()
}

/// Round 12 item 1 (`PORTING-PLAN.md` -- this round-set's self-side same-pair
/// breakdown): of the 62 seed-20260804 `right_arm --collision` cases where
/// this backend and the oracle agree on the self-side deepest pair but
/// disagree on its magnitude, 52 are the known `base_bellow_link`/
/// `torso_lift_link` plateau (deviation 6(b), `PORTING-PLAN.md` §56/§63.1).
/// The other 10 are `base_link`/one of five distinct `*_caster_*_wheel_link`
/// links -- the same pair family `parry.rs`'s deviation-6 doc already names
/// as producing this backend's own frozen `-0.046592m` constant (a shared
/// planar `base_link` mesh face, not wheel symmetry -- see that doc and
/// `pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face`,
/// below). Of those 10, 3 have an oracle magnitude exceeding twice the
/// wheel's own bounding radius (`link_bounding_radius`, `0.0767m` for a pr2
/// caster wheel, so a `0.1534m` bound) -- geometrically impossible for that
/// pair, on either backend, the same
/// failure mode `panda_worst_sweep_deviation_is_not_a_missed_deeper_contact`
/// already documents for deviation 6(a). The remaining 7 stay within that
/// bound and are not adjudicated by it either way: a within-bound number is
/// consistent with both "this backend misses a real deeper contact" and
/// "another independent-EPA local-minimum disagreement". Round 13 found a
/// backend-independent ground truth for this pair *family* specifically --
/// `parry3d_f64`'s own per-triangle `query::contact`, called directly
/// against each of `base_link`'s 96 mesh triangles rather than through
/// `distance_self`'s full pipeline (`pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face`)
/// -- but that only explains *this backend's own* `-0.046592m`, not whether
/// the oracle's FCL/libccd search missed something deeper; a self-side pair
/// still has no fixed external reference the way `floor_env`'s box gives
/// `deepest_vertex_under_floor`, so the oracle side of the remaining 7
/// still needs a case-by-case investigation this test does not attempt.
///
/// Round 15 re-ran the same op against today's tree with two seeds
/// (20260804/999002, the 62-case round-12 sweep was never committed as a
/// fixture and so cannot be re-selected by index) and found the same split
/// at a larger, independently reproduced sample: 5 of 21 caster-wheel
/// same-pair cases exceed the `0.1534m` bound, 16 stay within it. The
/// within-bound remainder's oracle side is still not adjudicated: closing
/// it needs FCL/libccd's own penetration-depth source, which is not
/// present locally (`~/work/moveit2` is `moveit2`, which depends on FCL,
/// not FCL itself) -- see `parry.rs`'s deviation-6(b) doc for the full
/// blocker writeup.
#[test]
fn pr2_self_wheel_same_pair_oracle_magnitude_is_implausible() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let acm = build_acm("pr2.srdf");
    let env = floor_env();

    for point in load_self_wheel_oracle_points() {
        let mut state = build_state(&model, &point.joint_values);
        let posed = state.update();
        let request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let distance = env.distance_self(&request, &posed, &[]);

        assert!(
            distance
                .minimum_distance
                .link_names
                .contains(&point.wheel_link_name)
                && distance
                    .minimum_distance
                    .link_names
                    .contains(&"base_link".to_owned()),
            "{}: oracle's argmin pair is base_link/{}, this backend's is {:?} -- expected \
             agreement on the pair, disagreement only on depth",
            point.wheel_link_name,
            point.wheel_link_name,
            distance.minimum_distance.link_names
        );

        let bound = link_bounding_radius(&model, &point.wheel_link_name)
            .map(|radius| 2.0 * radius)
            .unwrap_or_else(|| {
                panic!(
                    "{}: expected a finite bounding radius",
                    point.wheel_link_name
                )
            });
        assert!(
            point.oracle_distance.abs() > bound,
            "{}: oracle's own {} no longer exceeds this pair's {bound} bound -- the implausible \
             number this test exists to document may have been fixed upstream (rebuild the \
             fixture) or this case no longer belongs in the implausible set",
            point.wheel_link_name,
            point.oracle_distance
        );
        assert!(
            distance.minimum_distance.distance.abs() <= bound,
            "{}: this backend's own {} exceeds its own plausibility bound {bound} -- a real \
             regression, not the oracle-side implausibility this test documents",
            point.wheel_link_name,
            distance.minimum_distance.distance
        );
    }
}

/// The winning `base_link` mesh triangle (its own vertex indices) against
/// `cylinder_link_name`'s cylinder, and the exact contact depth
/// `parry3d_f64::query::contact` finds for it -- computed by calling that
/// same query directly against each of `base_link`'s individual mesh
/// triangles (in the cylinder's own local frame, so both shapes sit at a
/// fixed relative pose independent of the caller's world frame), rather
/// than through `distance_self`'s full pipeline. `TriMesh`'s own narrow
/// phase (`contact_composite_shape_shape`, see
/// `panda_worst_sweep_deviation_is_not_a_missed_deeper_contact`'s doc for
/// how this was confirmed from the vendored source) already visits every
/// overlapping triangle exhaustively and keeps the deepest, so this
/// reproduces `distance_self`'s own per-pair answer exactly while also
/// naming *which* triangle produced it -- the "backend-independent ground
/// truth" `pr2_self_wheel_same_pair_oracle_magnitude_is_implausible`'s own
/// doc says a self-side pair has none of, using `parry3d_f64`'s own EPA as
/// the reference rather than reimplementing it. (A hand-rolled point-to-
/// surface distance was tried first and rejected: for a large mesh
/// triangle piercing clean through a solid cylinder, the true EPA
/// minimum-separating-translation depth is generally *not* recoverable as
/// any single point's distance to the cylinder's surface -- that pointwise
/// measure only equals the true penetration depth for an infinite
/// half-space, as `deepest_vertex_under_floor` relies on, not a bounded
/// convex cylinder.)
fn native_deepest_triangle_vs_cylinder(
    model: &RobotModel,
    posed: &cspace_core::state::Posed<'_, '_>,
    mesh_link_name: &str,
    cylinder_link_name: &str,
) -> (f64, [u32; 3]) {
    let mesh_link = model
        .link_model(mesh_link_name)
        .unwrap_or_else(|e| panic!("{mesh_link_name}: {e}"));
    let mesh_shape = &mesh_link.shapes()[0];
    let Shape::Mesh(mesh) = &mesh_shape.shape else {
        panic!("{mesh_link_name} shape[0] is not a mesh");
    };
    let mesh_frame = posed
        .global_link_transform(mesh_link_name)
        .unwrap_or_else(|e| panic!("{mesh_link_name}: {e}"))
        * mesh_shape.origin_transform;

    let cyl_link = model
        .link_model(cylinder_link_name)
        .unwrap_or_else(|e| panic!("{cylinder_link_name}: {e}"));
    let cyl_shape = &cyl_link.shapes()[0];
    let Shape::Cylinder(cylinder) = &cyl_shape.shape else {
        panic!("{cylinder_link_name} shape[0] is not a cylinder");
    };
    let cyl_frame = posed
        .global_link_transform(cylinder_link_name)
        .unwrap_or_else(|e| panic!("{cylinder_link_name}: {e}"))
        * cyl_shape.origin_transform;

    let to_cyl = cyl_frame.inverse() * mesh_frame;
    let local_vertices: Vec<parry3d_f64::math::Vector> = mesh
        .vertices
        .iter()
        .map(|v| {
            let p = to_cyl.transform_point(&nalgebra::Point3::from(*v));
            parry3d_f64::math::Vector::new(p.x, p.y, p.z)
        })
        .collect();

    let parry_cylinder = parry3d_f64::shape::Cylinder::new(cylinder.length * 0.5, cylinder.radius);
    // parry's canonical cylinder axis is Y; `to_cyl` above already expresses
    // every point in the cylinder shape's *own* local frame (Z along its
    // axis, matching `convert_shape`'s `axis_fix`), so querying parry's
    // `Cylinder` primitive needs that same Y-onto-Z rotation applied to its
    // own pose (identity otherwise).
    let axis_fix: parry3d_f64::math::Pose =
        nalgebra::Isometry3::rotation(nalgebra::Vector3::x() * std::f64::consts::FRAC_PI_2).into();
    let identity: parry3d_f64::math::Pose = nalgebra::Isometry3::identity().into();

    let mut best = f64::INFINITY;
    let mut best_tri = [0u32; 3];
    for tri in &mesh.triangles {
        let p0 = local_vertices[tri[0] as usize];
        let p1 = local_vertices[tri[1] as usize];
        let p2 = local_vertices[tri[2] as usize];
        let triangle = parry3d_f64::shape::Triangle::new(p0, p1, p2);
        let Ok(Some(contact)) =
            parry3d_f64::query::contact(&identity, &triangle, &axis_fix, &parry_cylinder, 0.0)
        else {
            continue;
        };
        if contact.dist < best {
            best = contact.dist;
            best_tri = *tri;
        }
    }
    (best, best_tri)
}

/// Round 13 item 2 (`PORTING-PLAN.md`): confirms, for all three
/// `pr2_self_wheel_same_pair_oracle_implausible_{request,response}.json`
/// cases (two `br_caster_l_wheel_link` poses, one `fl_caster_l_wheel_link`
/// pose), that this backend's own frozen `-0.046592m` self-distance
/// constant is [`native_deepest_triangle_vs_cylinder`]'s answer too, and
/// that the winning triangle is the *same* `base_link` mesh triangle
/// (vertex indices `[14, 12, 15]`) every time, with all three of its
/// vertices sharing one `z` in `base_link`'s own frame -- the load-bearing
/// fact behind `parry.rs`'s corrected deviation-6 doc: a near-planar face
/// of `base_link`'s own coarse collision mesh, not a wheel symmetry.
#[test]
fn pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face() {
    const TOLERANCE: f64 = 1e-9;
    const PLANAR_TOLERANCE: f64 = 1e-9;

    let model = build_model("pr2.urdf", "pr2.srdf");
    let acm = build_acm("pr2.srdf");
    let env = floor_env();

    for point in load_self_wheel_oracle_points() {
        let mut state = build_state(&model, &point.joint_values);
        let posed = state.update();

        let (native_depth, winning_triangle) = native_deepest_triangle_vs_cylinder(
            &model,
            &posed,
            "base_link",
            &point.wheel_link_name,
        );

        // `Global` (default), not `Single`: `Single` tracks a search bound
        // per pair key, so with PR2's ~40 links every one of the ~800 other
        // pairs pays an unbounded-threshold query with no benefit from this
        // pair's own result (`DistanceRequestType::Single`'s own doc has the
        // measurement -- 1000x+ slower for this exact case). `Global`'s
        // shared, shrinking bound prunes those in comparison, and gives the
        // identical answer for *this* pair, since
        // `pr2_self_wheel_same_pair_oracle_magnitude_is_implausible` already
        // establishes base_link/this wheel is the true global self-distance
        // minimum at these fixture points, not merely *a* pair among many.
        let request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let distance = env.distance_self(&request, &posed, &[]);
        assert!(
            distance
                .minimum_distance
                .link_names
                .contains(&"base_link".to_owned())
                && distance
                    .minimum_distance
                    .link_names
                    .contains(&point.wheel_link_name),
            "{}: expected the global self-distance minimum to be the base_link/wheel pair \
             this test is about, found {:?} instead -- Global's minimum_distance no longer \
             names the pair Single's distances[key] used to, so the two request types are not \
             interchangeable here anymore",
            point.wheel_link_name,
            distance.minimum_distance.link_names,
        );
        let backend_depth = distance.minimum_distance.distance;

        assert!(
            (native_depth - backend_depth).abs() < TOLERANCE,
            "{}: native per-triangle search found {native_depth}, distance_self found \
             {backend_depth} -- expected the same answer, since TriMesh's own narrow phase \
             already visits every overlapping triangle exhaustively",
            point.wheel_link_name,
        );
        assert_eq!(
            winning_triangle,
            [14, 12, 15],
            "{}: a different base_link triangle won this time -- the shared-planar-face \
             explanation for the frozen constant may no longer hold",
            point.wheel_link_name,
        );

        let mesh_link = model.link_model("base_link").unwrap();
        let Shape::Mesh(mesh) = &mesh_link.shapes()[0].shape else {
            panic!("base_link shape[0] is not a mesh");
        };
        let z0 = mesh.vertices[winning_triangle[0] as usize][2];
        for idx in winning_triangle {
            let z = mesh.vertices[idx as usize][2];
            assert!(
                (z - z0).abs() < PLANAR_TOLERANCE,
                "{}: winning triangle vertex {idx} has z={z}, expected {z0} -- the triangle is \
                 not planar in base_link's own frame, so it cannot explain a joint-invariant depth",
                point.wheel_link_name,
            );
        }
    }
}

/// Round 13 item 2's own one-parameter-family sweep (`PORTING-PLAN.md`):
/// with every other joint fixed at
/// `pr2_self_wheel_same_pair_oracle_implausible_request.json` case 3's own
/// values, sweeps `fl_caster_rotation_joint` -- whose axis (vertical, `0 0
/// 1`) is nowhere near `fl_caster_l_wheel_link`'s own (horizontal) roll
/// axis -- and confirms the *actual* shape of this backend's own answer as
/// a function of that one joint: a `min` of (at least) three candidate
/// `base_link` triangles. One is
/// [`pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face`]'s
/// planar `[14, 12, 15]`, and it does dominate at every interior point
/// sampled here -- but a second, genuinely `theta`-varying candidate
/// (triangle `[13, 12, 14]`) is shallower near `theta = 0` and deepens
/// monotonically approaching the plateau. This is the same "one constant
/// candidate, one that actually moves, `min` of the two" shape `parry.rs`'s
/// deviation 6(b) already documents for `base_bellow_link`/
/// `torso_lift_link`'s `candidate_x`/`candidate_z(t)`, not a fully global
/// invariant: a dense 72-point sweep (this test's own scratch predecessor,
/// not committed) found the planar candidate's plateau covers roughly 80%
/// of the joint's full range, and two *different* triangles (`[13, 12,
/// 14]` near `theta = 0`, `[15, 12, 16]` near `theta ~ 3.5..4.4`) win the
/// remaining ~20%. This backend's own `-0.046592m` constant is therefore
/// this pair's plateau, not its value everywhere -- the same distinction
/// [`pr2_self_wheel_same_pair_oracle_magnitude_is_implausible`]'s own doc
/// comment already draws for `base_bellow_link`/`torso_lift_link`. Had the
/// plateau covered the *entire* range, or had the ramp region stayed
/// frozen too, either would have been the real defect this test exists to
/// catch.
#[test]
fn pr2_self_wheel_same_pair_frozen_constant_is_a_plateau_not_a_global_invariant() {
    const TOLERANCE: f64 = 1e-9;
    const PLATEAU_TRIANGLE: [u32; 3] = [14, 12, 15];
    const PLATEAU_DEPTH: f64 = -0.046592;
    const RAMP_TRIANGLE: [u32; 3] = [13, 12, 14];

    let model = build_model("pr2.urdf", "pr2.srdf");
    let base_points = load_self_wheel_oracle_points();
    let base_joint_values = base_points[2].joint_values.clone();
    assert_eq!(base_points[2].wheel_link_name, "fl_caster_l_wheel_link");

    let query_at = |theta: f64| {
        let mut joint_values = base_joint_values.clone();
        joint_values.insert("fl_caster_rotation_joint".to_owned(), theta);
        let mut state = build_state(&model, &joint_values);
        let posed = state.update();
        native_deepest_triangle_vs_cylinder(&model, &posed, "base_link", "fl_caster_l_wheel_link")
    };

    // Well inside the plateau (a dense 72-point sweep found it solid across
    // roughly theta in [0.52, 3.40] union [4.45, 5.76]): the same triangle,
    // the same depth, at every point.
    for theta in [0.7, 1.6, 2.4, 3.1, 4.6, 5.0, 5.5] {
        let (depth, triangle) = query_at(theta);
        assert_eq!(
            triangle, PLATEAU_TRIANGLE,
            "theta={theta}: expected the plateau triangle to still win here"
        );
        assert!(
            (depth - PLATEAU_DEPTH).abs() < TOLERANCE,
            "theta={theta}: depth {depth} left the plateau's {PLATEAU_DEPTH}"
        );
    }

    // Near theta=0, a *different* triangle wins, with a depth that
    // genuinely moves -- monotonically deepening as theta approaches the
    // plateau -- rather than being frozen at some other constant. Proving
    // this is what distinguishes "this pair is a plateau" from "this pair
    // is frozen and the plateau assertions above just got lucky."
    let ramp_points: Vec<(f64, f64, [u32; 3])> = [0.0, 0.1, 0.2, 0.3, 0.4]
        .into_iter()
        .map(|theta| {
            let (depth, triangle) = query_at(theta);
            (theta, depth, triangle)
        })
        .collect();
    for (theta, _, triangle) in &ramp_points {
        assert_eq!(
            *triangle, RAMP_TRIANGLE,
            "theta={theta}: expected the ramp triangle, not the plateau one, this close to theta=0"
        );
    }
    for pair in ramp_points.windows(2) {
        let (theta_a, depth_a, _) = pair[0];
        let (theta_b, depth_b, _) = pair[1];
        assert!(
            depth_b < depth_a,
            "theta={theta_a}->{theta_b}: depth {depth_a}->{depth_b} did not strictly deepen \
             approaching the plateau"
        );
    }
    assert!(
        ramp_points.last().unwrap().1 > PLATEAU_DEPTH,
        "the ramp's last sampled point already reached the plateau's own depth -- sample closer \
         to theta=0 so this test still exercises the ramp, not the plateau"
    );
}

/// Replicates `cspace_planning::constraints`' `VisibilityConstraint::cone_mesh` exact
/// vertex/triangle formula (see that method's own doc comment: vertex `0`
/// sensor origin, vertex `1` target center, vertices `2..cone_sides+2` the
/// disc rim) so this crate can drive the same mesh through its own
/// `parry3d_f64::query::contact` without a dependency on
/// `cspace_planning::constraints` -- that crate already depends on this one
/// (`Cargo.toml`), so the reverse edge would be a cycle.
fn visibility_cone_mesh_world(
    world_to_sensor: &Isometry3,
    world_to_target: &Isometry3,
    target_radius: f64,
    cone_sides: usize,
) -> (Vec<nalgebra::Vector3<f64>>, Vec<[u32; 3]>) {
    let mut vertices = Vec::with_capacity(cone_sides + 2);
    vertices.push(world_to_sensor.translation.vector);
    vertices.push(world_to_target.translation.vector);
    let delta = 2.0 * std::f64::consts::PI / cone_sides as f64;
    for i in 0..cone_sides {
        let a = delta * i as f64;
        let rim_point_in_target =
            nalgebra::Vector3::new(a.sin() * target_radius, a.cos() * target_radius, 0.0);
        vertices.push((world_to_target * nalgebra::Point3::from(rim_point_in_target)).coords);
    }

    let mut triangles = Vec::with_capacity(cone_sides * 2);
    for i in 1..cone_sides {
        triangles.push([(i + 1) as u32, 0, (i + 2) as u32]);
        triangles.push([(i + 1) as u32, 1, (i + 2) as u32]);
    }
    triangles.push([(cone_sides + 1) as u32, 0, 2]);
    triangles.push([(cone_sides + 1) as u32, 1, 2]);
    (vertices, triangles)
}

/// Same triangle-vs-cylinder query [`native_deepest_triangle_vs_cylinder`]
/// runs for a URDF mesh link's own STL, but against an in-memory cone mesh
/// ([`visibility_cone_mesh_world`]) instead -- so a visibility-cone
/// near-placement case can be probed the same way without going through
/// `cspace_planning::constraints`/`tools/moveit-diff`. Returns the winning triangle's
/// depth and indices, plus the target-center vertex (mesh vertex `1`)
/// expressed in the cylinder's own local frame, for the caller to check it
/// landed at the local origin.
fn deepest_cone_triangle_vs_cylinder(
    cyl_frame: &Isometry3,
    cylinder: &cspace_core::geometry::Cylinder,
    vertices: &[nalgebra::Vector3<f64>],
    triangles: &[[u32; 3]],
) -> (f64, [u32; 3], nalgebra::Point3<f64>) {
    let to_cyl = cyl_frame.inverse();
    let local_vertices: Vec<parry3d_f64::math::Vector> = vertices
        .iter()
        .map(|v| {
            let p = to_cyl.transform_point(&nalgebra::Point3::from(*v));
            parry3d_f64::math::Vector::new(p.x, p.y, p.z)
        })
        .collect();
    let target_center_local = to_cyl.transform_point(&nalgebra::Point3::from(vertices[1]));

    let parry_cylinder = parry3d_f64::shape::Cylinder::new(cylinder.length * 0.5, cylinder.radius);
    let axis_fix: parry3d_f64::math::Pose =
        nalgebra::Isometry3::rotation(nalgebra::Vector3::x() * std::f64::consts::FRAC_PI_2).into();
    let identity: parry3d_f64::math::Pose = nalgebra::Isometry3::identity().into();

    let mut best = f64::INFINITY;
    let mut best_tri = [0u32; 3];
    for tri in triangles {
        let p0 = local_vertices[tri[0] as usize];
        let p1 = local_vertices[tri[1] as usize];
        let p2 = local_vertices[tri[2] as usize];
        let triangle = parry3d_f64::shape::Triangle::new(p0, p1, p2);
        let Ok(Some(contact)) =
            parry3d_f64::query::contact(&identity, &triangle, &axis_fix, &parry_cylinder, 0.0)
        else {
            continue;
        };
        if contact.dist < best {
            best = contact.dist;
            best_tri = *tri;
        }
    }
    (best, best_tri, target_center_local)
}

/// Round 25 (`PORTING-PLAN.md`): generalizes case 104's own single-case
/// finding across a sample of `(target_radius, cone_sides)` pairs spanning
/// `visibility_cone_depth_sweep.rs`'s own near-branch ranges (`target_radius`
/// in `0.005..0.015`, `cone_sides` in `3..=8`, `build_case`'s
/// `Some(link_name)` arm) crossed with all three of
/// [`load_self_wheel_oracle_points`]'s real pr2 joint states (two
/// `br_caster_l_wheel_link` poses, one `fl_caster_l_wheel_link` pose) -- 15
/// combinations, not only case 104's one.
///
/// Two claims were on the table; measuring all 15 splits them:
///
/// - The near-placement's own construction (`build_case`/`tools/moveit-diff`'s
///   `build_constraint_case`: target pose exactly at `link_fk *
///   shape.origin_transform`) puts the cone's vertex 1 (the target-center
///   vertex) at the cylinder's own local origin, and keeps every cone vertex
///   inside the cylinder's own inscribed sphere (`target_radius`'s sampled
///   `0.005..=0.015` stays under every sampled wheel's own `min(radius,
///   length / 2)`), guaranteeing real penetration regardless of orientation.
///   **This generalizes**: measured exactly (`target_center_local` within
///   float noise of the origin, `depth < 0.0`) in all 15 combinations below,
///   matching `parry.rs`'s deviation-6(b) doc's "every such case
///   interpenetrates through the link's own centroid by construction".
/// - That the *winning* (max-depth) triangle specifically contains vertex 1,
///   the way case 104's own `[5, 1, 6]` did. **This does not generalize**:
///   measured true in only 4 of these 15 combinations (both wins are pinned
///   below by the total, not by which specific `(joint_state, radius,
///   cone_sides)` triples they were, since that triple set is not itself a
///   stable fact worth freezing) -- the rest won through a triangle sharing
///   the *sensor* vertex (vertex 0) instead. Case 104's own `[5, 1, 6]` was
///   this mechanism's most visible instance, not its general shape; not
///   asserted here as a per-case rule because it is false as one.
#[test]
fn visibility_cone_near_placement_interpenetrates_through_the_touched_links_own_centroid() {
    const ORIGIN_TOLERANCE: f64 = 1e-9;
    const SENSOR_OFFSET: f64 = 0.005;
    const SAMPLES: &[(f64, usize)] = &[(0.005, 3), (0.0075, 4), (0.01, 5), (0.0125, 6), (0.015, 8)];

    let model = build_model("pr2.urdf", "pr2.srdf");
    let mut cases_checked = 0;
    let mut winning_triangle_had_vertex_one = 0;

    for point in load_self_wheel_oracle_points() {
        let mut state = build_state(&model, &point.joint_values);
        let posed = state.update();

        let link = model
            .link_model(&point.wheel_link_name)
            .unwrap_or_else(|e| panic!("{}: {e}", point.wheel_link_name));
        let shape = &link.shapes()[0];
        let Shape::Cylinder(cylinder) = &shape.shape else {
            panic!("{} shape[0] is not a cylinder", point.wheel_link_name);
        };
        let cyl_frame = posed
            .global_link_transform(&point.wheel_link_name)
            .unwrap_or_else(|e| panic!("{}: {e}", point.wheel_link_name))
            * shape.origin_transform;
        let anchor = cyl_frame.translation.vector;
        let world_to_target =
            Isometry3::from_parts(anchor.into(), nalgebra::UnitQuaternion::identity());
        let world_to_sensor = Isometry3::from_parts(
            (anchor + nalgebra::Vector3::new(0.0, 0.0, SENSOR_OFFSET)).into(),
            nalgebra::UnitQuaternion::identity(),
        );

        for &(target_radius, cone_sides) in SAMPLES {
            let (vertices, triangles) = visibility_cone_mesh_world(
                &world_to_sensor,
                &world_to_target,
                target_radius,
                cone_sides,
            );
            let (depth, winning_triangle, target_center_local) =
                deepest_cone_triangle_vs_cylinder(&cyl_frame, cylinder, &vertices, &triangles);

            assert!(
                target_center_local.coords.norm() < ORIGIN_TOLERANCE,
                "{}: target-center vertex {target_center_local:?} did not land at the \
                 cylinder's own local origin (radius={target_radius}, cone_sides={cone_sides})",
                point.wheel_link_name
            );
            assert!(
                depth < 0.0,
                "{}: expected a real penetration, got depth {depth} (radius={target_radius}, \
                 cone_sides={cone_sides})",
                point.wheel_link_name
            );
            if winning_triangle.contains(&1) {
                winning_triangle_had_vertex_one += 1;
            }
            cases_checked += 1;
        }
    }

    assert_eq!(
        cases_checked,
        load_self_wheel_oracle_points().len() * SAMPLES.len(),
        "sanity: every joint-state x (radius, cone_sides) combination above ran"
    );
    assert_eq!(
        winning_triangle_had_vertex_one, 4,
        "the winning-triangle-contains-vertex-1 rate moved from this round's measured 4/15 -- \
         update this test's own doc comment (and any claim-audit prose citing it) to match, it \
         is describing a measured fact, not an enforced invariant"
    );
}

/// The one pair in this fixture set whose correct separation distance is
/// known in closed form, checked against that closed form rather than
/// against the oracle.
///
/// Every other distance claim in this workspace is comparative: it says the
/// port and the oracle agree, or names why they do not. That cannot settle
/// which side is right when they disagree by a little, and PORTING-PLAN.md
/// §5 Phase 3's `distance: f64` clause turns on exactly such a residual --
/// the sweep's separated-branch worst on pr2 is `6.056201e-07`, far under
/// the clause's `1e-4` but far over the machine precision the mesh-only
/// fixtures reach. This pair decides it without a second implementation.
///
/// The geometry is fixed by `fixtures/pr2.urdf` and [`floor_env`] alone:
///
/// - the floor is `Cuboid::new(4.0, 4.0, 0.1)` -- full extents, not half --
///   centred at `z = -0.05`, so its top face is the plane `z = 0`;
/// - `*_caster_*_wheel_link`'s collision geometry is a cylinder of radius
///   `0.074792`, and its centre sits at `z = 0.051 + 0.0282 + 0.0 = 0.0792`
///   above the root (`base_footprint_joint`, `*_caster_rotation_joint`,
///   `*_caster_*_wheel_joint`).
///
/// So the separation is `0.0792 - 0.074792 = 0.004408` exactly, and it is
/// *pose-invariant*: `*_caster_rotation_joint` turns about `z` and cannot
/// change a centre height, `*_caster_*_wheel_joint` turns the cylinder about
/// its own axis, and a cylinder is invariant under that. Sweeping both over
/// a full turn therefore has one correct answer, the same one every time.
///
/// Measured before the tolerance was chosen: over the 24 poses below this
/// backend's worst deviation from that constant is `7.766010e-14`, so
/// `CLOSED_FORM_TOL` is pinned at `1e-12` -- a 12.9x margin over the
/// measurement, not a number borrowed from a neighbouring test. For
/// comparison the oracle's own published value on this pair, case 6338 of
/// the seed-1 10,000-state sweep, is `4.40860562000265372e-3`, which misses
/// the same constant by `6.056200e-07`: about 7.8 million times this
/// backend's error. That is the measurement behind the claim that the
/// separated branch's residual belongs to the reference rather than to this
/// port, and it is why widening the clause to absorb it would be the wrong
/// move -- there is nothing here to absorb.
///
/// The argmin pair is asserted too. Which of the eight wheels wins is a tie
/// broken by pose (all eight appear across these 24), but if the winner ever
/// stops being a caster wheel against `floor` this test would be silently
/// measuring some other pair's distance against a constant derived for this
/// one.
#[test]
fn pr2_caster_wheel_floor_clearance_matches_the_closed_form() {
    /// Pinned from the measurement in this test's doc comment
    /// (`7.766010e-14` worst over the same 24 poses), with a 12.9x margin.
    const CLOSED_FORM_TOL: f64 = 1e-12;

    let model = build_model("pr2.urdf", "pr2.srdf");
    let acm = build_acm("pr2.srdf");
    let env = floor_env();

    // `0.0792` is the wheel centre's height above the root and `0.074792`
    // its cylinder radius, both read off `fixtures/pr2.urdf` above.
    let expected = 0.0792 - 0.074792;

    let mut poses = 0;
    for step in 0..24 {
        let angle = f64::from(step) * std::f64::consts::TAU / 24.0;
        let mut joint_values = BTreeMap::new();
        for side in ["fl", "fr", "bl", "br"] {
            // Three different multiples so the steering angle and the two
            // wheel angles are never equal -- a single shared angle would
            // pass even if two of the three were being ignored.
            joint_values.insert(format!("{side}_caster_rotation_joint"), angle);
            joint_values.insert(format!("{side}_caster_l_wheel_joint"), -angle);
            joint_values.insert(format!("{side}_caster_r_wheel_joint"), angle * 2.0);
        }
        let mut state = build_state(&model, &joint_values);
        let posed = state.update();
        let request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let result = env.distance_robot(&request, &posed, &[]);
        let names = &result.minimum_distance.link_names;

        assert!(
            names.contains(&"floor".to_owned())
                && names
                    .iter()
                    .any(|n| n.contains("_caster_") && n.ends_with("_wheel_link")),
            "step {step}: the nearest robot/world pair is {names:?}, no longer a caster wheel \
             against floor -- the closed form below was derived for that pair and would be \
             measuring something else"
        );
        let deviation = (result.minimum_distance.distance - expected).abs();
        assert!(
            deviation <= CLOSED_FORM_TOL,
            "step {step}: caster clearance is {} but the geometry fixes it at {expected} \
             (deviation {deviation:.6e} over the pinned {CLOSED_FORM_TOL:.0e}); this is a \
             closed-form answer, so a miss here is this backend's",
            result.minimum_distance.distance
        );
        poses += 1;
    }

    assert_eq!(poses, 24, "sanity: every pose above ran");
}

/// The second pair in this fixture set whose correct separation distance is
/// known in closed form -- and the first where the closed form contradicts the
/// oracle by more than PORTING-PLAN.md §5 Phase 3's own `1e-4`.
///
/// [`pr2_caster_wheel_floor_clearance_matches_the_closed_form`] settled a
/// `6.1e-7` residual the same way; this settles a `1.7e-3` one. The sweep that
/// raised it is `tools/moveit-diff` on prbt with the floor lowered off the
/// mounting tangency (`--cases 10000 --seed 1 --floor-top-z -0.5`), whose one
/// separated-branch failure in 19,611 comparisons is case 8148:
///
/// ```text
/// collision[8148] robot |d| 1.683122e-3
///   oracle 3.11769210552093334e-1 [floor(world_object)/prbt_flange(robot_link)]
///   vs rust 3.10086089038992263e-1 [prbt_flange(robot_link)/floor(world_object)]
/// ```
///
/// Both sides name the same pair, so this is a two-shape distance disagreement
/// with nothing else in it. The geometry is fixed by `fixtures/prbt.urdf` and
/// [`floor_env_with_top`] alone:
///
/// - the floor is `Cuboid::new(4.0, 4.0, 0.1)` -- full extents, not half --
///   centred at `z = -0.55`, so its top face is the plane `z = -0.5`;
/// - `prbt_flange`'s collision geometry is a cylinder of `length 0.02` and
///   `radius 0.0331` at `<origin xyz="0 0 -0.0035">` in the link frame
///   (`fixtures/prbt.urdf`'s `prbt_flange` link).
///
/// While the cylinder is above that face and its whole silhouette projects
/// inside the 4x4 footprint -- both asserted per state below, because the
/// closed form is only the answer while the nearest box feature is the face --
/// the exact distance for a cylinder of half-length `h`, radius `r`, centre `c`
/// and unit axis `a` is `c_z - z0 - h * |a_z| - r * sqrt(1 - a_z^2)`.
///
/// `c` and `a` come from *this port's* forward kinematics, which needs saying:
/// §5 Phase 2's fk condition is MET at `1e-9` against the oracle, so the pose
/// is not the thing under test here. It is also checkable directly for the
/// pinned case -- evaluating the same closed form on the oracle's own `fk`
/// answer for case 8148 gives `3.10086088255497272e-1` against this port's
/// `3.10086088255497827e-1`, a difference of `5.551115e-16`. The `1.683122e-3`
/// therefore cannot be kinematics.
///
/// Measured before the tolerance was chosen: over the 40 poses below this
/// backend's worst deviation from the closed form is `1.427251e-8`, so
/// `CLOSED_FORM_TOL` is pinned at `1e-7` -- a 7.0x margin over the
/// measurement. Case 8148 itself is `7.834950e-10`, pinned separately at
/// `1e-8` (12.8x), because that is the number the sweep row is about.
///
/// The reference's own error is `1.683122e-3`, about 2.1 million times this
/// backend's on the same state, and `tools/ci/verify-fcl-cylinder-box-distance.sh`
/// re-derives where it comes from inside the pinned oracle image: bare
/// `fcl::distance` on these two shapes answers `3.11769210552093334e-1` with
/// the box passed first and `3.10086138896653651e-1` with the cylinder passed
/// first, and tightening only the GJK stopping threshold collapses both onto
/// the closed form. See `doc/upstream-bugs.md`'s
/// `distance-callback-default-tolerance-makes-distance-order-dependent`.
#[test]
fn prbt_flange_floor_clearance_matches_the_closed_form() {
    /// Pinned from the measurement in this test's doc comment
    /// (`1.427251e-8` worst over the same 40 poses), with a 7.0x margin.
    const CLOSED_FORM_TOL: f64 = 1e-7;
    /// The same, for the one pinned state (`7.834950e-10`), with 12.8x.
    const CASE_8148_TOL: f64 = 1e-8;
    /// What the oracle published for `floor`/`prbt_flange` at case 8148. The
    /// sweep printed `3.11769210552093334e-1`; that is one digit past what an
    /// `f64` can carry, and this is the same value written so the literal is
    /// exactly the number the compiler stores.
    const CASE_8148_ORACLE: f64 = 0.311_769_210_552_093_33;
    /// PORTING-PLAN.md §5 Phase 3's distance tolerance, the bar the oracle's
    /// value has to miss for the sweep row to be a real disagreement.
    const CLAUSE_TOL: f64 = 1e-4;
    /// `fixtures/prbt.urdf`'s `prbt_flange` <collision>, and the scene.
    const CYL_RADIUS: f64 = 0.0331;
    const CYL_HALF_LENGTH: f64 = 0.01;
    const CYL_ORIGIN_Z: f64 = -0.0035;
    const FLOOR_TOP_Z: f64 = -0.5;
    const FLOOR_HALF_EXTENT: f64 = 2.0;

    let model = build_model("prbt.urdf", "prbt.srdf");
    let env = floor_env_with_top(FLOOR_TOP_Z);

    // Everything allowed except the one pair the closed form is derived for,
    // so `minimum_distance` is that pair's distance rather than whichever
    // link happens to be lowest. Without it a state where `prbt_link_5` wins
    // would be scored against a constant derived for the flange.
    let mut names: Vec<String> = model
        .link_models()
        .iter()
        .map(|link| link.name().to_owned())
        .collect();
    names.push("floor".to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    acm.set_entry("floor", "prbt_flange", false);

    // Case 8148's own joint values, as `tools/moveit-diff --stats-json`
    // recorded them for that state.
    let case_8148 = [
        -0.681765976663856,
        -1.9643143747676026,
        1.9502553948473182,
        -0.2497111438317039,
        -0.47123744163653836,
        -1.186060955153117,
    ];

    let mut poses = 0;
    let mut saw_case_8148 = false;
    for step in 0..40 {
        // A scan through case 8148's own joint vector rather than around it:
        // `scale == 1.0` at step 20 *is* case 8148, and every other step moves
        // all six joints at once, so a state cannot pass by one joint being
        // ignored.
        let scale = 0.4 + f64::from(step) * 0.03;
        let joint_values: BTreeMap<String, f64> = case_8148
            .iter()
            .enumerate()
            .map(|(i, value)| (format!("prbt_joint_{}", i + 1), value * scale))
            .collect();
        let mut state = build_state(&model, &joint_values);
        let posed = state.update();

        let flange = posed
            .global_link_transform("prbt_flange")
            .expect("prbt has a prbt_flange link");
        let centre = (flange * Isometry3::translation(0.0, 0.0, CYL_ORIGIN_Z))
            .translation
            .vector;
        let axis = flange.rotation * cspace_core::geometry::Vector3::new(0.0, 0.0, 1.0);

        // The closed form is the distance to the *plane* the top face lies in,
        // so it is the distance to the box only while no cylinder point is
        // nearer a box edge. `reach` is the largest horizontal offset of any
        // cylinder point from the centre, `CYL_HALF_LENGTH + CYL_RADIUS` the
        // largest vertical one.
        let reach = CYL_HALF_LENGTH * axis[0].hypot(axis[1]) + CYL_RADIUS;
        assert!(
            centre[0].abs() + reach < FLOOR_HALF_EXTENT
                && centre[1].abs() + reach < FLOOR_HALF_EXTENT
                && centre[2] - (CYL_HALF_LENGTH + CYL_RADIUS) > FLOOR_TOP_Z,
            "step {step}: the flange cylinder at {centre:?} (axis {axis:?}) is not strictly \
             above the floor's top face and inside its footprint, so the plane closed form \
             below is not the distance to the box"
        );

        let expected = centre[2]
            - FLOOR_TOP_Z
            - CYL_HALF_LENGTH * axis[2].abs()
            - CYL_RADIUS * (1.0 - axis[2] * axis[2]).max(0.0).sqrt();

        let request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let result = env.distance_robot(&request, &posed, &[]);
        let names = &result.minimum_distance.link_names;
        assert!(
            names.contains(&"floor".to_owned()) && names.contains(&"prbt_flange".to_owned()),
            "step {step}: the nearest robot/world pair is {names:?}, not floor/prbt_flange -- \
             the closed form below was derived for that pair and would be measuring something \
             else"
        );

        let deviation = (result.minimum_distance.distance - expected).abs();
        assert!(
            deviation <= CLOSED_FORM_TOL,
            "step {step}: flange clearance is {} but the geometry fixes it at {expected} \
             (deviation {deviation:.6e} over the pinned {CLOSED_FORM_TOL:.0e}); this is a \
             closed-form answer, so a miss here is this backend's",
            result.minimum_distance.distance
        );

        if scale == 1.0 {
            saw_case_8148 = true;
            assert!(
                deviation <= CASE_8148_TOL,
                "case 8148: this port is {deviation:.6e} from the closed form, over the pinned \
                 {CASE_8148_TOL:.0e}"
            );
            // The other half of the sweep row, and the reason this test is not
            // just another parity check: the reference's published value for
            // this very pair misses the same closed form by more than the
            // clause tolerates, so the `1.683122e-3` is the reference's error
            // and there is nothing here for a wider tolerance to absorb.
            let oracle_deviation = (CASE_8148_ORACLE - expected).abs();
            assert!(
                oracle_deviation > CLAUSE_TOL,
                "case 8148: the oracle's published {CASE_8148_ORACLE} is only \
                 {oracle_deviation:.6e} from the closed form, inside the clause's \
                 {CLAUSE_TOL:.0e} -- the divergence this test exists to attribute is gone"
            );
        }
        poses += 1;
    }

    assert_eq!(poses, 40, "sanity: every pose above ran");
    assert!(saw_case_8148, "sanity: step 20 is case 8148 itself");
}

/// PORTING-PLAN.md §281.6's third residual bullet said the closed form
/// above "only holds on the separated side". It does not: raising the floor
/// above case 8148's own flange pose turns the SAME formula negative -- a
/// penetration-depth prediction rather than a clearance -- and this backend's
/// `distance_robot` matches it there too, to a tighter tolerance than the
/// separated branch above.
///
/// `FLOOR_TOP_Z` is raised from -0.5 to -0.16 -- 0.34m above
/// [`prbt_flange_floor_clearance_matches_the_closed_form`]'s floor, chosen so
/// case 8148's own pose (`scale == 1.0`) penetrates by about 3cm, well inside
/// the box's 0.05m half-thickness margin the footprint/depth assertions below
/// require (so the top face, not the bottom one, stays the nearest feature --
/// the same "closed form is only the answer while ..." caveat the separated
/// test states, extended to a symmetric depth bound). The scan is the same
/// case-8148-relative scale sweep as that test, restricted to the eight steps
/// (15..=22, scale 0.85 to 1.06) whose closed-form value is actually negative
/// at this floor height; the other 32 steps are that test's territory, not
/// this one's.
///
/// MEASURED over those eight poses before the tolerance was chosen: worst
/// deviation `1.583803e-15` (step 15, scale 0.85), the deepest state (step 19,
/// scale 0.97, depth `3.361721e-2`) at `1.804112e-16`. `CLOSED_FORM_TOL` is
/// pinned at `1e-14`, a 6.3x margin -- tighter than the separated branch's
/// `1e-7` because there is no GJK iteration to converge here, just the same
/// closed-form arithmetic evaluated at a shifted plane.
#[test]
fn prbt_flange_floor_clearance_matches_the_closed_form_when_penetrating() {
    /// Pinned from the measurement in this test's doc comment
    /// (`1.583803e-15` worst over the eight penetrating poses), with a 6.3x
    /// margin.
    const CLOSED_FORM_TOL: f64 = 1e-14;
    /// `fixtures/prbt.urdf`'s `prbt_flange` <collision>, and the scene.
    const CYL_RADIUS: f64 = 0.0331;
    const CYL_HALF_LENGTH: f64 = 0.01;
    const CYL_ORIGIN_Z: f64 = -0.0035;
    /// Raised from -0.5 (the separated test's floor) so case 8148's own pose
    /// penetrates by roughly 3cm -- see this test's doc comment.
    const FLOOR_TOP_Z: f64 = -0.16;
    const FLOOR_HALF_EXTENT: f64 = 2.0;
    /// `floor_env_with_top`'s own box thickness; the closed form is only the
    /// distance to the *top face* while the penetration depth stays under
    /// half of this, so the bottom face cannot be nearer.
    const FLOOR_HALF_THICKNESS: f64 = 0.05;

    let model = build_model("prbt.urdf", "prbt.srdf");
    let env = floor_env_with_top(FLOOR_TOP_Z);

    let mut names: Vec<String> = model
        .link_models()
        .iter()
        .map(|link| link.name().to_owned())
        .collect();
    names.push("floor".to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    acm.set_entry("floor", "prbt_flange", false);

    // Case 8148's own joint values -- same array
    // [`prbt_flange_floor_clearance_matches_the_closed_form`] scales.
    let case_8148 = [
        -0.681765976663856,
        -1.9643143747676026,
        1.9502553948473182,
        -0.2497111438317039,
        -0.47123744163653836,
        -1.186060955153117,
    ];

    let mut poses = 0;
    let mut saw_case_8148 = false;
    for step in 15..=22 {
        // Same scale formula as the separated test, so step numbers line up
        // with its own (step == 20 is still case 8148 itself, scale 1.0).
        let scale = 0.4 + f64::from(step) * 0.03;
        let joint_values: BTreeMap<String, f64> = case_8148
            .iter()
            .enumerate()
            .map(|(i, value)| (format!("prbt_joint_{}", i + 1), value * scale))
            .collect();
        let mut state = build_state(&model, &joint_values);
        let posed = state.update();

        let flange = posed
            .global_link_transform("prbt_flange")
            .expect("prbt has a prbt_flange link");
        let centre = (flange * Isometry3::translation(0.0, 0.0, CYL_ORIGIN_Z))
            .translation
            .vector;
        let axis = flange.rotation * cspace_core::geometry::Vector3::new(0.0, 0.0, 1.0);

        let reach = CYL_HALF_LENGTH * axis[0].hypot(axis[1]) + CYL_RADIUS;
        let expected = centre[2]
            - FLOOR_TOP_Z
            - CYL_HALF_LENGTH * axis[2].abs()
            - CYL_RADIUS * (1.0 - axis[2] * axis[2]).max(0.0).sqrt();
        assert!(
            centre[0].abs() + reach < FLOOR_HALF_EXTENT
                && centre[1].abs() + reach < FLOOR_HALF_EXTENT
                && expected < 0.0
                && expected > -FLOOR_HALF_THICKNESS,
            "step {step}: the flange cylinder at {centre:?} (axis {axis:?}) is not strictly \
             inside the floor's footprint and penetrating the top face by less than half its \
             thickness, so the plane closed form below is not the distance to the box"
        );

        let request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let result = env.distance_robot(&request, &posed, &[]);
        let names = &result.minimum_distance.link_names;
        assert!(
            names.contains(&"floor".to_owned()) && names.contains(&"prbt_flange".to_owned()),
            "step {step}: the nearest robot/world pair is {names:?}, not floor/prbt_flange -- \
             the closed form below was derived for that pair and would be measuring something \
             else"
        );
        assert!(
            result.minimum_distance.distance < 0.0,
            "step {step}: expected a penetrating (negative) signed distance, got {} -- this pose \
             is no longer in the penetration branch this test exists to check",
            result.minimum_distance.distance
        );

        let deviation = (result.minimum_distance.distance - expected).abs();
        assert!(
            deviation <= CLOSED_FORM_TOL,
            "step {step}: flange penetration is {} but the geometry fixes it at {expected} \
             (deviation {deviation:.6e} over the pinned {CLOSED_FORM_TOL:.0e}); this is a \
             closed-form answer, so a miss here is this backend's",
            result.minimum_distance.distance
        );

        if step == 20 {
            saw_case_8148 = true;
        }
        poses += 1;
    }

    assert_eq!(poses, 8, "sanity: every penetrating pose above ran");
    assert!(
        saw_case_8148,
        "sanity: step 20 is case 8148 itself, scaled to 1.0"
    );
}

/// One convex collision shape, already placed in world coordinates, reduced
/// to the two closed-form primitives [`convex_distance_bracket`] needs.
///
/// Deliberately not a general shape type, and deliberately only the three
/// variants its callers reach: the bracket below is a *proof* only for convex
/// bodies whose support function and point projection are exact. A mesh has
/// both, but only up to its own triangulation, and calling that a third
/// answer would be circular. Every other `Shape` panics in
/// [`WorldConvex::from_link_shape`] rather than being added unused, so a new
/// caller has to state which shapes it means.
///
/// `Sphere` is here because prbt's separated tail needs it: `prbt_link_3` and
/// `prbt_link_5` each carry one, and which of a link's shapes realises the
/// minimum is exactly what
/// [`prbt_separated_tail_splits_on_the_curved_generic_gjk_cell`] has to
/// measure rather than assume. Without the variant that test cannot tell a
/// `cylinder x sphere` minimum from the `cylinder x cylinder` beside it, and
/// those two cells fall on opposite sides of its partition.
#[derive(Clone)]
enum WorldConvex {
    Box {
        centre: cspace_core::geometry::Vector3,
        /// World directions of the box's own three axes, unit length.
        axes: [cspace_core::geometry::Vector3; 3],
        /// Half-extents along `axes`, i.e. `Cuboid::size` halved.
        half: [f64; 3],
    },
    Cylinder {
        centre: cspace_core::geometry::Vector3,
        /// World direction of the cylinder's local `+z`, unit length.
        axis: cspace_core::geometry::Vector3,
        half_length: f64,
        radius: f64,
    },
    Sphere {
        centre: cspace_core::geometry::Vector3,
        radius: f64,
    },
}

impl WorldConvex {
    /// `link_shape` placed by `link_pose`, or a panic naming the shape kind.
    ///
    /// Panics rather than returning `None`: a caller that silently skipped an
    /// unsupported shape would compute the bracket over a *subset* of a
    /// link's geometry and report it as the link's distance, which is the one
    /// failure mode that would look like a pass.
    fn from_link_shape(link_pose: &Isometry3, link_shape: &cspace_core::model::LinkShape) -> Self {
        let pose = link_pose * link_shape.origin_transform;
        let centre = pose.translation.vector;
        let dir = |x, y, z| pose.rotation * cspace_core::geometry::Vector3::new(x, y, z);
        match &link_shape.shape {
            Shape::Cuboid(b) => Self::Box {
                centre,
                axes: [dir(1.0, 0.0, 0.0), dir(0.0, 1.0, 0.0), dir(0.0, 0.0, 1.0)],
                half: [b.size[0] * 0.5, b.size[1] * 0.5, b.size[2] * 0.5],
            },
            Shape::Cylinder(c) => Self::Cylinder {
                centre,
                axis: dir(0.0, 0.0, 1.0),
                half_length: c.length * 0.5,
                radius: c.radius,
            },
            Shape::Sphere(s) => Self::Sphere {
                centre,
                radius: s.radius,
            },
            other => panic!(
                "convex_distance_bracket has no exact support function for {other:?}; the \
                 bracket it produces would not be a proof"
            ),
        }
    }

    /// `max_{x in S} x . n`, exact for both variants.
    fn support_max(&self, n: &cspace_core::geometry::Vector3) -> f64 {
        match self {
            Self::Box { centre, axes, half } => {
                centre.dot(n) + (0..3).map(|k| half[k] * axes[k].dot(n).abs()).sum::<f64>()
            }
            Self::Cylinder {
                centre,
                axis,
                half_length,
                radius,
            } => {
                let along = axis.dot(n);
                centre.dot(n)
                    + half_length * along.abs()
                    + radius * (1.0 - along * along).max(0.0).sqrt()
            }
            Self::Sphere { centre, radius } => centre.dot(n) + radius,
        }
    }

    /// `min_{x in S} x . n`, by the same closed form on `-n`.
    fn support_min(&self, n: &cspace_core::geometry::Vector3) -> f64 {
        -self.support_max(&(-n))
    }

    /// The point of `S` nearest `p` -- exact for both variants (clamp in the
    /// box's own frame; clamp radially and axially in the cylinder's).
    fn project(&self, p: &cspace_core::geometry::Vector3) -> cspace_core::geometry::Vector3 {
        match self {
            Self::Box { centre, axes, half } => {
                let d = p - centre;
                let mut out = *centre;
                for k in 0..3 {
                    out += axes[k] * d.dot(&axes[k]).clamp(-half[k], half[k]);
                }
                out
            }
            Self::Cylinder {
                centre,
                axis,
                half_length,
                radius,
            } => {
                let d = p - centre;
                let along = d.dot(axis).clamp(-*half_length, *half_length);
                let radial = d - axis * d.dot(axis);
                let norm = radial.norm();
                let radial = if norm > *radius {
                    radial * (*radius / norm)
                } else {
                    radial
                };
                centre + axis * along + radial
            }
            Self::Sphere { centre, radius } => {
                let d = p - centre;
                let norm = d.norm();
                if norm > *radius {
                    centre + d * (*radius / norm)
                } else {
                    *p
                }
            }
        }
    }

    fn centre(&self) -> cspace_core::geometry::Vector3 {
        match self {
            Self::Box { centre, .. }
            | Self::Cylinder { centre, .. }
            | Self::Sphere { centre, .. } => *centre,
        }
    }

    /// A point of `S` attaining `max_{x in S} x . n`, exact for all three
    /// variants.
    ///
    /// This is the *witness* behind [`WorldConvex::support_max`] and it is what
    /// makes [`minkowski_depth_bracket`]'s lower bound second-order rather than
    /// first-order: `x . m <= h_S(m)` holds for every `m`, not just for `n`, so
    /// one evaluation certifies a whole spherical cap instead of only its
    /// centre. `support_point(n) . n == support_max(n)` by construction, which
    /// [`world_convex_support_point_attains_the_support_value`] checks.
    ///
    /// Where the maximiser is not unique the choice is free -- every maximiser
    /// gives the same `x . n`, and a non-extreme choice (a cylinder's axis
    /// point when `n` is along its axis) is still a point of the body, so the
    /// bound it certifies stays valid.
    fn support_point(&self, n: &cspace_core::geometry::Vector3) -> cspace_core::geometry::Vector3 {
        match self {
            Self::Box { centre, axes, half } => {
                let mut out = *centre;
                for k in 0..3 {
                    let s = if axes[k].dot(n) >= 0.0 { 1.0 } else { -1.0 };
                    out += axes[k] * (half[k] * s);
                }
                out
            }
            Self::Cylinder {
                centre,
                axis,
                half_length,
                radius,
            } => {
                let along = axis.dot(n);
                let s = if along >= 0.0 { 1.0 } else { -1.0 };
                // The radial direction is built inside an orthonormal frame of
                // `axis` rather than by normalising `n - axis (axis . n)`.
                // That subtraction cancels to rounding scale when `n` is along
                // the axis, and normalising the remainder returns a direction
                // whose *own* axis component is O(1) -- which puts the point
                // outside the cylinder and makes the bound it certifies false.
                // Here `e1` and `e2` are perpendicular to `axis` by
                // construction, so every point produced is on the rim whatever
                // the rounding does, and a degenerate `n` costs accuracy
                // (bounded by `radius * m`, itself at rounding scale) instead
                // of soundness.
                let seed = if axis[0].abs() < 0.9 {
                    cspace_core::geometry::Vector3::new(1.0, 0.0, 0.0)
                } else {
                    cspace_core::geometry::Vector3::new(0.0, 1.0, 0.0)
                };
                let e1 = (seed - axis * axis.dot(&seed)).normalize();
                let e2 = axis.cross(&e1);
                let (c1, c2) = (n.dot(&e1), n.dot(&e2));
                let m = (c1 * c1 + c2 * c2).sqrt();
                let radial = if m > 0.0 {
                    (e1 * c1 + e2 * c2) * (*radius / m)
                } else {
                    cspace_core::geometry::Vector3::zeros()
                };
                centre + axis * (half_length * s) + radial
            }
            Self::Sphere { centre, radius } => {
                let norm = n.norm();
                if norm > 0.0 {
                    centre + n * (*radius / norm)
                } else {
                    *centre
                }
            }
        }
    }

    /// An upper bound on `|x - centre|` over `x in S`, exact for all three.
    ///
    /// Used only by [`signed_distance_floor`], which needs a *bound* rather
    /// than the true circumradius: overstating it makes that floor weaker, so
    /// it prunes fewer pairs and costs time, and cannot make it prune one it
    /// should have kept.
    fn circumradius(&self) -> f64 {
        match self {
            Self::Box { half, .. } => {
                (half[0] * half[0] + half[1] * half[1] + half[2] * half[2]).sqrt()
            }
            Self::Cylinder {
                half_length,
                radius,
                ..
            } => (half_length * half_length + radius * radius).sqrt(),
            Self::Sphere { radius, .. } => *radius,
        }
    }

    /// The `shapes::ShapeType` name fcl's specialisation table is indexed by,
    /// for reporting which cell a measured minimum lands in.
    fn kind(&self) -> &'static str {
        match self {
            Self::Box { .. } => "box",
            Self::Cylinder { .. } => "cylinder",
            Self::Sphere { .. } => "sphere",
        }
    }
}

/// A certified `[lower, upper]` bracket on `dist(a, b)` for two disjoint
/// convex bodies, and the witness pair that produced it.
///
/// This is what stands in for a closed form on a shape pair that has none.
/// PORTING-PLAN.md §260.8 recorded that prbt's separated-branch residual
/// could not be settled the way pr2's caster was, because "box 대 cylinder의
/// 분리 거리는 자세에 따라 변하므로 상수가 없다" -- and that is still true:
/// the minimising features here are the cylinder's bottom rim *circle* and an
/// *edge* of the box, whose common perpendicular is the root of a quartic, so
/// there is no short formula to write down. What there is instead is a proof
/// with two closed-form halves, neither of which depends on the answer being
/// searched for correctly:
///
///   * **upper**, `|p - q|` for *any* `p` in `a` and `q` in `b`. Every pair of
///     points on the two bodies is an upper bound on their distance.
///   * **lower**, `min_{x in a} x.n - max_{y in b} y.n` for *any* unit `n`.
///     Every direction separates the two bodies by at most their distance
///     (weak duality between the two support functions).
///
/// So the returned interval contains `dist(a, b)` whatever the alternating
/// projection below did -- the iteration is only a way to *find* a tight
/// pair, and the interval's width is the measure of how tight, checked by the
/// caller rather than assumed. A run that converged badly reports a wide
/// bracket and fails its caller's assertion; it cannot report a wrong answer
/// as a narrow one.
fn convex_distance_bracket(a: &WorldConvex, b: &WorldConvex) -> DistanceBracket {
    let mut p = a.project(&b.centre());
    for _ in 0..256 {
        let q = b.project(&p);
        let next = a.project(&q);
        if (next - p).norm() <= f64::EPSILON {
            p = next;
            break;
        }
        p = next;
    }
    let q = b.project(&p);
    let gap = p - q;
    let upper = gap.norm();
    // A zero-length gap means the bodies touch or overlap, where the dual
    // direction is undefined; the caller's own separation assertion is what
    // rules that out, and a zero-width bracket keeps this function total.
    if upper == 0.0 {
        return DistanceBracket {
            lower: 0.0,
            upper: 0.0,
            on_a: p,
            on_b: q,
        };
    }
    let n = gap / upper;
    DistanceBracket {
        lower: a.support_min(&n) - b.support_max(&n),
        upper,
        on_a: p,
        on_b: q,
    }
}

/// [`convex_distance_bracket`]'s result: the two bounds and the witness pair
/// that produced them.
///
/// The witnesses are carried out because *which features* realise the minimum
/// is the reason this pair has no short closed form, and a claim about that
/// belongs on the same instrument as the numbers rather than in a scratch
/// script beside it.
struct DistanceBracket {
    lower: f64,
    upper: f64,
    /// The witness point on the first body, in world coordinates.
    on_a: cspace_core::geometry::Vector3,
    /// The witness point on the second body, in world coordinates.
    on_b: cspace_core::geometry::Vector3,
}

/// A certified `[lower, upper]` bracket on the **signed** distance between two
/// convex bodies, negative when they overlap, and the direction that produced
/// the upper bound.
///
/// [`convex_distance_bracket`] cannot cross zero: it brackets `|p - q|`, which
/// is `0` for every overlapping pair however deep. This is the other half, and
/// it is the instrument PORTING-PLAN.md §297.5 said the tree did not have --
/// the one thing §297.4's `249/389` depth disagreement needs before it can be
/// charged to a side.
///
/// The quantity is the one both implementations claim to report: the minimum
/// translation distance. For convex `A`, `B` write `D = A - B` (the Minkowski
/// difference, `{a - b}`), whose support function is exactly
/// `h_D(n) = h_A(n) + h_B(-n)` -- two closed forms this file already has. `D`
/// is convex, and `D = {x : x.n <= h_D(n) for all unit n}`, so the largest ball
/// about the origin contained in `D` has radius `min_{|n|=1} h_D(n)`. That
/// radius is the depth by which `A` and `B` must be pulled apart, and the same
/// expression is `-dist(A, B)` when they are disjoint, so one function covers
/// both branches with one sign convention.
///
/// What makes the result a proof rather than a search:
///
///   * **upper on the depth** -- `h_D(n)` for *any* unit `n` is an upper bound
///     on the minimum, so every direction ever evaluated tightens it and none
///     can corrupt it.
///   * **lower on the depth** -- for any point `x` of `D`, `h_D(m) >= x.m` for
///     *every* `m`. Taking `x = supp_D(n)` and a spherical cap of angular
///     radius `rho` about `n`, `x.m >= |x| cos(theta + rho)` where `theta` is
///     the angle between `x` and `n`. That bounds the whole cap from one
///     evaluation, and it is second-order accurate at the minimiser (where `x`
///     and `n` are parallel, so `cos` is flat), which is why the branch and
///     bound below closes in a few thousand evaluations instead of the `1e9`
///     patches a plain Lipschitz net would need at these tolerances.
///
/// The branch and bound refines the most promising patch first and stops on
/// `TOL` or on `MAX_EVALS`. Either way it returns the bracket it actually
/// achieved -- a run that ran out of budget reports a wide interval, and the
/// caller's own rule about what width it will judge on is what turns that into
/// a verdict or a `too wide to judge`. It cannot report a wrong answer
/// narrowly.
struct DepthBracket {
    /// Certified lower bound on `min_{|n|=1} h_D(n)`.
    lower: f64,
    /// Certified upper bound on the same, i.e. the best `h_D(n)` seen.
    upper: f64,
    /// The unit direction attaining `upper`. For an overlapping pair this is
    /// the minimum-translation axis.
    axis: cspace_core::geometry::Vector3,
    /// Support evaluations spent. Carried so a caller can tell a bracket that
    /// converged from one that hit the budget.
    evals: usize,
}

impl DepthBracket {
    /// The bracket restated in MoveIt's sign convention: negative when the
    /// bodies overlap.
    ///
    /// `distanceCallback` and `parry3d_f64::query::contact` both report a
    /// penetrating pair as a negative distance, so a comparison against either
    /// has to negate -- and negating swaps the two ends.
    fn signed(&self) -> (f64, f64) {
        (-self.upper, -self.lower)
    }

    fn width(&self) -> f64 {
        self.upper - self.lower
    }
}

/// A cheap certified lower bound on the signed distance between two bodies,
/// for skipping pairs that cannot hold a *minimum* over many pairs.
///
/// `dist(a, b) >= |ca - cb| - Ra - Rb` for the circumradii, and the signed
/// distance is never below `-(Ra + Rb)`, so this floor holds on both branches.
/// A caller looking for the smallest signed distance across a robot's link
/// pairs can drop any pair whose floor already exceeds the best value it has,
/// without running [`minkowski_depth_bracket`] on it at all -- which is what
/// makes a whole-population sweep affordable. It prunes on a *bound*, so a
/// pair it drops provably could not have been the minimum.
fn signed_distance_floor(a: &WorldConvex, b: &WorldConvex) -> f64 {
    (a.centre() - b.centre()).norm() - a.circumradius() - b.circumradius()
}

/// Support evaluations a single [`minkowski_depth_bracket`] call may spend when
/// the caller wants its converged answer.
///
/// Not reached on any pair of prbt's population once the caller runs a coarse
/// pass first: see [`SCOUT_BUDGET`].
const FULL_BUDGET: usize = 200_000;

/// Support evaluations for the *scouting* pass over a link pair's shapes.
///
/// The point of a coarse pass is only to find one pair whose certified `lower`
/// is large, so every other pair can be dropped against it. `512` is four
/// levels of subdivision past the icosahedron on the active branch, which was
/// enough on prbt's population to pick the winning pair in every case.
const SCOUT_BUDGET: usize = 512;

/// Default absolute bracket width [`minkowski_depth_bracket`] stops at. Five
/// orders below the `1e-4` the §5 Phase 3 clause is written at, so a bracket
/// that reaches it can arbitrate any disagreement that clause would call a
/// failure. A caller measuring what a tighter bracket would resolve (see
/// §302.9) passes a narrower value directly rather than through this
/// constant.
const TOL: f64 = 1e-9;

/// The smallest signed distance over `pairs`, as a certified bracket, with the
/// pair that realises it.
///
/// This is the aggregation MoveIt's `distance_robot` performs and it is where
/// the branch and bound of [`minkowski_depth_bracket`] gets its cutoff. Two
/// passes: scout every candidate cheaply, take the largest depth any of them
/// *certifies*, then refine only those that can still beat it. Dropping a pair
/// against a certified bound is sound -- the dropped pair's depth is provably
/// below one already achieved -- so the result is the same bracket the
/// exhaustive refinement gives, which
/// [`prbt_penetration_branch_is_bracketed_by_the_minkowski_instrument`] pins
/// on the cases where the two answers disagree.
///
/// Both ends aggregate by `min`: `min` of the true values is at least the `min`
/// of the lower bounds and at most the `min` of the upper bounds, so the
/// interval is still a proof after aggregation, even though it can be wider
/// than any single pair's.
fn min_signed_distance_over(pairs: &[(WorldConvex, WorldConvex)], tol: f64) -> (f64, f64, usize) {
    // Deepest-looking first, by the free floor, so the scout certifies a large
    // depth early and the rest drop against it.
    let mut order: Vec<usize> = (0..pairs.len()).collect();
    order.sort_by(|&i, &j| {
        signed_distance_floor(&pairs[i].0, &pairs[i].1)
            .total_cmp(&signed_distance_floor(&pairs[j].0, &pairs[j].1))
    });

    let mut certified_depth = f64::NEG_INFINITY;
    let mut scouted: Vec<(usize, f64)> = Vec::new();
    for &i in &order {
        // `depth <= -floor` always, so a pair whose floor already rules out the
        // depth some other pair has certified needs no evaluation at all.
        if -signed_distance_floor(&pairs[i].0, &pairs[i].1) < certified_depth {
            continue;
        }
        let d = minkowski_depth_bracket(
            &pairs[i].0,
            &pairs[i].1,
            f64::NEG_INFINITY,
            SCOUT_BUDGET,
            TOL,
        );
        certified_depth = certified_depth.max(d.lower);
        scouted.push((i, d.upper));
    }

    let (mut lo, mut hi) = (f64::INFINITY, f64::INFINITY);
    let mut winner = usize::MAX;
    for (i, upper) in scouted {
        // `upper` bounds this pair's depth from above; a pair that cannot reach
        // a depth another pair already certifies cannot hold the minimum signed
        // distance either.
        if upper < certified_depth {
            continue;
        }
        let d = minkowski_depth_bracket(&pairs[i].0, &pairs[i].1, -hi, FULL_BUDGET, tol);
        let (slo, shi) = d.signed();
        if shi < hi {
            hi = shi;
            winner = i;
        }
        lo = lo.min(slo);
    }
    (lo, hi, winner)
}

/// The bracket of [`DepthBracket`], by branch and bound over the unit sphere.
///
/// `TOL` and `MAX_EVALS` are the stopping rule, not the accuracy claim: the
/// accuracy claim is the returned `lower`/`upper` pair, which is valid at every
/// point of the search.
///
/// `cutoff` is the caller's branch and bound, one level up. A caller looking
/// for the *deepest* of many pairs passes the depth it has already achieved;
/// this search abandons as soon as its own upper bound falls below that, since
/// the pair can no longer win. The bracket it returns then is wide but still
/// certified, and both of its ends provably sit above the running minimum, so
/// aggregating it changes nothing. Pass `f64::NEG_INFINITY` to disable.
///
/// `max_evals` bounds the search. A pair that does not converge inside it
/// returns its achieved width rather than a tighter claim, so the budget
/// trades width for time and never correctness. It exists so a caller can
/// bracket every candidate cheaply, take the largest certified depth, and
/// spend the real budget only on pairs that can still beat it.
///
/// MEASURED, on the 389-case population: with neither the cutoff nor a cheap
/// first pass, 315 of 389 cases had a shape pair spend the entire budget --
/// nearly all of them *separated* pairs stalling at a width of `1.5e-9` to
/// `4.6e-9` against a `TOL` of `1e-9`, pairs whose signed distance is positive
/// and which could not have held a penetration minimum at all.
fn minkowski_depth_bracket(
    a: &WorldConvex,
    b: &WorldConvex,
    cutoff: f64,
    max_evals: usize,
    tol: f64,
) -> DepthBracket {
    type V3 = cspace_core::geometry::Vector3;

    // `h_D` and its witness, the only two places the geometry enters.
    let h = |n: &V3| a.support_max(n) + b.support_max(&(-n));
    let witness = |n: &V3| a.support_point(n) - b.support_point(&(-n));

    /// One spherical triangle of the working subdivision.
    /// The triangle's own cap -- centre `(v0+v1+v2)/|.|` and cosine
    /// `min_i v[i] . centre` -- provably covers it: for `n = w/|w|` with `w` in
    /// the planar triangle, `n . centre = (w . centre)/|w| >= w . centre >=
    /// min_i v[i] . centre` whenever that minimum is non-negative and
    /// `|w| <= 1`, both of which hold for an icosahedral face and every
    /// subdivision of one. Only `lower` survives the construction, because
    /// splitting needs the vertices and nothing else.
    struct Patch {
        v: [V3; 3],
        lower: f64,
    }

    // Ordered by `lower` *reversed*, so `BinaryHeap`'s max is the frontier's
    // minimum certified bound. `total_cmp` rather than `partial_cmp`: a NaN
    // bound would otherwise make the ordering inconsistent and `BinaryHeap`
    // silently return the wrong patch instead of panicking.
    impl Ord for Patch {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            other.lower.total_cmp(&self.lower)
        }
    }
    impl PartialOrd for Patch {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    impl PartialEq for Patch {
        fn eq(&self, other: &Self) -> bool {
            self.cmp(other) == std::cmp::Ordering::Equal
        }
    }
    impl Eq for Patch {}

    let mut evals = 0usize;
    let mut best_upper = f64::INFINITY;
    let mut best_axis = V3::new(0.0, 0.0, 1.0);

    let mut eval = |n: &V3, evals: &mut usize, best_upper: &mut f64, best_axis: &mut V3| -> V3 {
        *evals += 1;
        let value = h(n);
        if value < *best_upper {
            *best_upper = value;
            *best_axis = *n;
        }
        witness(n)
    };

    // The cap bound of the doc comment, for one witness against one cap.
    let cap_bound = |x: &V3, centre: &V3, cos_rho: f64| -> f64 {
        let norm = x.norm();
        if norm == 0.0 {
            // The origin is a point of `D`, so `h_D >= 0` everywhere and the
            // bodies touch at worst. Nothing sharper is available from it.
            return 0.0;
        }
        let cos_theta = (x.dot(centre) / norm).clamp(-1.0, 1.0);
        let angle = cos_theta.acos() + cos_rho.clamp(-1.0, 1.0).acos();
        if angle >= std::f64::consts::PI {
            -norm
        } else {
            norm * angle.cos()
        }
    };

    let make_patch = |v: [V3; 3],
                      evals: &mut usize,
                      best_upper: &mut f64,
                      best_axis: &mut V3,
                      eval: &mut dyn FnMut(&V3, &mut usize, &mut f64, &mut V3) -> V3|
     -> Patch {
        let centre = (v[0] + v[1] + v[2]).normalize();
        let cos_rho = v
            .iter()
            .map(|x| x.dot(&centre))
            .fold(f64::INFINITY, f64::min);
        // A patch wider than a hemisphere would break the containment argument
        // above; `-1` makes the cap the whole sphere, which is still sound and
        // simply prunes nothing.
        let cos_rho = if cos_rho >= 0.0 { cos_rho } else { -1.0 };
        let mut lower = f64::NEG_INFINITY;
        for n in std::iter::once(&centre).chain(v.iter()) {
            let x = eval(n, evals, best_upper, best_axis);
            lower = lower.max(cap_bound(&x, &centre, cos_rho));
        }
        Patch { v, lower }
    };

    // A regular icosahedron's 20 faces, built from its 12 vertices so the seed
    // subdivision is uniform and no direction starts with a privileged patch.
    let phi = (1.0 + 5.0_f64.sqrt()) * 0.5;
    let raw = [
        [-1.0, phi, 0.0],
        [1.0, phi, 0.0],
        [-1.0, -phi, 0.0],
        [1.0, -phi, 0.0],
        [0.0, -1.0, phi],
        [0.0, 1.0, phi],
        [0.0, -1.0, -phi],
        [0.0, 1.0, -phi],
        [phi, 0.0, -1.0],
        [phi, 0.0, 1.0],
        [-phi, 0.0, -1.0],
        [-phi, 0.0, 1.0],
    ];
    let vertices: Vec<V3> = raw
        .iter()
        .map(|c| V3::new(c[0], c[1], c[2]).normalize())
        .collect();
    const FACES: [[usize; 3]; 20] = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    let mut active: Vec<Patch> = FACES
        .iter()
        .map(|f| {
            make_patch(
                [vertices[f[0]], vertices[f[1]], vertices[f[2]]],
                &mut evals,
                &mut best_upper,
                &mut best_axis,
                &mut eval,
            )
        })
        .collect();

    // A min-heap on `lower`, so the frontier's smallest certified bound is both
    // the patch worth splitting next and the run's global lower bound. A linear
    // scan for it is quadratic in the frontier and was the whole cost of the
    // search when this was first run over prbt's population.
    let mut heap: std::collections::BinaryHeap<Patch> = active.drain(..).collect();

    while evals < max_evals {
        // The caller already has a deeper pair; nothing this one can still
        // certify would change its answer.
        if best_upper < cutoff {
            break;
        }
        let Some(front) = heap.peek() else { break };
        if best_upper - front.lower <= tol {
            break;
        }
        // Best-first: split only the patch that currently certifies the least,
        // so the budget goes where the bracket is actually held open.
        let p = heap.pop().expect("peeked");
        // `best_upper` may have improved since this patch was pushed; a patch
        // that can no longer hold the minimum is dropped rather than split.
        if p.lower > best_upper {
            continue;
        }
        let mid = |x: &V3, y: &V3| (x + y).normalize();
        let m = [
            mid(&p.v[0], &p.v[1]),
            mid(&p.v[1], &p.v[2]),
            mid(&p.v[2], &p.v[0]),
        ];
        for child in [
            [p.v[0], m[0], m[2]],
            [p.v[1], m[1], m[0]],
            [p.v[2], m[2], m[1]],
            [m[0], m[1], m[2]],
        ] {
            let patch = make_patch(
                child,
                &mut evals,
                &mut best_upper,
                &mut best_axis,
                &mut eval,
            );
            // Dropping a child above `best_upper` keeps the heap's minimum a
            // valid global bound: everything dropped certifies more than a
            // value already achieved.
            if patch.lower <= best_upper {
                heap.push(patch);
            }
        }
    }

    let lower = heap
        .peek()
        .map(|p| p.lower)
        .unwrap_or(best_upper)
        .min(best_upper);
    DepthBracket {
        lower,
        upper: best_upper,
        axis: best_axis,
        evals,
    }
}

/// [`WorldConvex::support_point`] returns a maximiser, not merely a point of
/// the body.
///
/// [`minkowski_depth_bracket`]'s lower bound is only valid if the witness it
/// takes really attains the support value: a point that is *inside* the body
/// still gives a sound-but-loose bound, whereas a point *outside* it would
/// certify a lower bound that is false. So the identity is checked directly
/// rather than argued from the closed forms, on every variant and on the
/// degenerate directions (along a box axis, along and across a cylinder's
/// axis) where the maximiser stops being unique.
#[test]
fn world_convex_support_point_attains_the_support_value() {
    let v = cspace_core::geometry::Vector3::new;
    let axis = |x: f64, y: f64, z: f64| v(x, y, z).normalize();
    let bodies = [
        WorldConvex::Box {
            centre: v(0.3, -0.2, 1.1),
            axes: [
                axis(1.0, 1.0, 0.0),
                axis(-1.0, 1.0, 0.0),
                axis(0.0, 0.0, 1.0),
            ],
            half: [0.121 / 2.0, 0.08 / 2.0, 0.17 / 2.0],
        },
        WorldConvex::Cylinder {
            centre: v(-0.4, 0.25, 0.6),
            axis: axis(0.2, -0.3, 1.0),
            half_length: 0.13,
            radius: 0.075,
        },
        WorldConvex::Sphere {
            centre: v(0.9, 0.1, -0.35),
            radius: 0.06,
        },
    ];
    // Deliberately includes the exact axis directions of each body above, so
    // the tie cases are covered rather than avoided by generic directions.
    let mut directions = vec![
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, 0.0, 1.0),
        v(-1.0, 0.0, 0.0),
        axis(1.0, 1.0, 0.0),
        axis(0.2, -0.3, 1.0),
        axis(-0.2, 0.3, -1.0),
        axis(0.3, 0.94, -0.16),
    ];
    // A deterministic spread, so the check is not only on hand-picked
    // directions. Golden-angle spiral: no seed, no rand dependency.
    let golden = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
    for i in 0..64 {
        let z = 1.0 - 2.0 * (i as f64 + 0.5) / 64.0;
        let r = (1.0 - z * z).max(0.0).sqrt();
        let t = golden * i as f64;
        directions.push(v(r * t.cos(), r * t.sin(), z));
    }

    let mut checked = 0usize;
    for body in &bodies {
        for n in &directions {
            let x = body.support_point(n);
            let want = body.support_max(n);
            let got = x.dot(n);
            assert!(
                (got - want).abs() <= 1e-12 * want.abs().max(1.0),
                "{} support point at {n:?} gives {got:.17e}, support value is {want:.17e} -- \
                 minkowski_depth_bracket's lower bound would be certified by a point that does \
                 not attain the maximum",
                body.kind()
            );
            checked += 1;
        }
    }
    assert_eq!(
        checked,
        bodies.len() * directions.len(),
        "every body x direction pair has to be checked; a skipped variant is a witness nobody \
         verified"
    );
}

/// [`minkowski_depth_bracket`] against the three configurations whose minimum
/// translation distance has a closed form, plus the disjoint case where it has
/// to reproduce [`convex_distance_bracket`].
///
/// Written because the branch and bound is the one part of this file that
/// searches: its bounds are proofs, but a bug in the subdivision could return
/// a *valid* bracket around the wrong quantity -- a bracket on the minimum over
/// only part of the sphere is still a bracket. Each case below fixes the
/// quantity independently of the search.
///
/// The disjoint case is the load-bearing one for §297.4's use: it shows the
/// two instruments in this file agree where their domains overlap, so the
/// depth verdicts and the separated verdicts are on one scale.
#[test]
fn minkowski_depth_bracket_matches_the_closed_forms_it_has() {
    let v = cspace_core::geometry::Vector3::new;

    /// How far outside the bracket a closed form may fall before the search is
    /// held responsible for it.
    ///
    /// Not a tolerance on the answer -- a rounding allowance between two float
    /// expressions for the same real. `h_D(n)` at an axis direction and
    /// `half_a + half_b - |ca - cb|` sum the same terms in different orders.
    /// MEASURED, by the `overshoot` line each of the three closed-form cases
    /// below prints: `0`, `0`, and `1.388e-17` (box x box, the only one whose
    /// closed form is a three-term subtraction). `16 * f64::EPSILON * |want|`
    /// is `1.954e-16` at that case's magnitude, 14x the observed worst, and
    /// still seven orders below the `1e-9` width the same case is required to
    /// reach. The fourth case, the disjoint pair, is checked against
    /// [`convex_distance_bracket`] rather than a closed form and does not use
    /// this constant.
    const CLOSED_FORM_SLACK: f64 = 16.0 * f64::EPSILON;

    let check = |name: &str, a: &WorldConvex, b: &WorldConvex, want: f64| {
        let bracket = minkowski_depth_bracket(a, b, f64::NEG_INFINITY, FULL_BUDGET, TOL);
        let slack = CLOSED_FORM_SLACK * want.abs();
        let overshoot = (bracket.lower - want).max(want - bracket.upper).max(0.0);
        println!(
            "{name}: want {want:.17e} bracket [{:.17e}, {:.17e}] width {:.3e} overshoot \
             {overshoot:.3e} slack {slack:.3e} evals {}",
            bracket.lower,
            bracket.upper,
            bracket.width(),
            bracket.evals
        );
        assert!(
            overshoot <= slack,
            "{name}: closed form {want:.17e} is outside the returned bracket \
             [{:.17e}, {:.17e}] by {overshoot:.3e}, over the {slack:.3e} rounding allowance \
             (width {:.3e}, {} evals) -- the branch and bound is not bracketing the minimum \
             over the whole sphere",
            bracket.lower,
            bracket.upper,
            bracket.width(),
            bracket.evals
        );
        assert!(
            bracket.width() <= 1e-9,
            "{name}: bracket width {:.3e} after {} evaluations, over the 1e-9 the search \
             stops at -- it ran out of budget on a case with a closed form",
            bracket.width(),
            bracket.evals
        );
    };

    // Two spheres, overlapping: depth is r1 + r2 - |c1 - c2| exactly, and the
    // minimising direction is the centre difference -- the one configuration
    // where the axis is known in closed form too, so it is where `axis` is
    // checked rather than taken on trust.
    let (c1, c2) = (v(0.0, 0.0, 0.0), v(0.05, 0.02, -0.01));
    let (r1, r2) = (0.07, 0.04);
    let (sa, sb) = (
        WorldConvex::Sphere {
            centre: c1,
            radius: r1,
        },
        WorldConvex::Sphere {
            centre: c2,
            radius: r2,
        },
    );
    check(
        "sphere x sphere, overlapping",
        &sa,
        &sb,
        r1 + r2 - (c2 - c1).norm(),
    );
    // `h_D(n) = (c1 - c2).n + r1 + r2` for two spheres, so the minimum is at
    // `n = (c2 - c1)/|c2 - c1|` -- pointing from the first body toward the
    // second, which is the direction the first has to be pushed back along.
    let want_axis = (c2 - c1).normalize();
    let got_axis = minkowski_depth_bracket(&sa, &sb, f64::NEG_INFINITY, FULL_BUDGET, TOL).axis;
    assert!(
        (got_axis - want_axis).norm() <= 1e-4,
        "sphere x sphere: minimum-translation axis {got_axis:?}, closed form {want_axis:?} -- \
         the reported axis is not the direction the reported depth was attained in"
    );

    // Sphere centre strictly inside a box: depth is r plus the centre's
    // distance to the nearest face.
    let bx = WorldConvex::Box {
        centre: v(0.1, 0.2, 0.3),
        axes: [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)],
        half: [0.06, 0.04, 0.085],
    };
    let sp_centre = v(0.11, 0.21, 0.34);
    let sp = WorldConvex::Sphere {
        centre: sp_centre,
        radius: 0.02,
    };
    let d = sp_centre - v(0.1, 0.2, 0.3);
    let face = [0.06 - d[0].abs(), 0.04 - d[1].abs(), 0.085 - d[2].abs()]
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    check("sphere inside box", &bx, &sp, 0.02 + face);

    // Two axis-aligned boxes: the Minkowski difference is a box, so the
    // minimum is the smallest per-axis overlap.
    let half_a = [0.121 / 2.0, 0.08 / 2.0, 0.17 / 2.0];
    let half_b = [0.09 / 2.0, 0.06 / 2.0, 0.12 / 2.0];
    let (ca, cb) = (v(0.0, 0.0, 0.0), v(0.04, 0.01, 0.09));
    let aligned = |centre, half: [f64; 3]| WorldConvex::Box {
        centre,
        axes: [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)],
        half,
    };
    let overlap = (0..3)
        .map(|k| half_a[k] + half_b[k] - (cb[k] - ca[k]).abs())
        .fold(f64::INFINITY, f64::min);
    assert!(
        overlap > 0.0,
        "the two boxes have to actually overlap for this case to be about depth; they clear by \
         {overlap:.3e}"
    );
    check(
        "box x box, axis aligned, overlapping",
        &aligned(ca, half_a),
        &aligned(cb, half_b),
        overlap,
    );

    // Disjoint: the same expression is -dist, so the two instruments in this
    // file have to agree. `convex_distance_bracket` is the reference here.
    let far = WorldConvex::Cylinder {
        centre: v(0.5, 0.0, 0.0),
        axis: v(0.0, 0.0, 1.0),
        half_length: 0.1,
        radius: 0.05,
    };
    let near = aligned(v(0.0, 0.0, 0.0), half_a);
    let separated = convex_distance_bracket(&near, &far);
    let depth = minkowski_depth_bracket(&near, &far, f64::NEG_INFINITY, FULL_BUDGET, TOL);
    let (signed_low, signed_high) = depth.signed();
    assert!(
        signed_low <= separated.upper && separated.lower <= signed_high,
        "disjoint box x cylinder: the depth instrument says the signed distance is in \
         [{signed_low:.17e}, {signed_high:.17e}] and the distance instrument says \
         [{:.17e}, {:.17e}] -- two proofs about the same pair that do not intersect means one \
         of them is not a proof",
        separated.lower,
        separated.upper
    );
    assert!(
        depth.upper < 0.0,
        "disjoint box x cylinder: the depth instrument reports {:.17e} >= 0, i.e. it calls a \
         separated pair overlapping",
        depth.upper
    );
}

/// §260.8's open item: prbt's separated-branch residual `8.892585e-5`,
/// attributed to one side or the other by a third answer that is neither
/// implementation's.
///
/// The residual is the worst separated-branch deviation of the whole prbt
/// sweep and it sits on one state -- `--stats-json`'s
/// `collision_clauses.separated.tail[0]`, case 4697, side `self`, pair
/// `prbt_link_4`/`prbt_base_link`, oracle `3.2876309670400866e-2` against
/// this port's `3.278738382005618e-2`. §260.4 already named that link pair;
/// what it got wrong is the *shape*. It says the worst value sits on
/// `prbt_link_4`'s `box 0.121 x 0.08 x 0.17`, and `fixtures/prbt.urdf` gives
/// that link two `<collision>` boxes. At case 4697 the first one is
/// `4.0199e-2` away from the base cylinder and the *second*, `0.09 x 0.06 x
/// 0.12`, is the one at `3.2787e-2`. Both are computed below, so the ranking
/// is measured here rather than asserted from that section.
///
/// The third answer is [`convex_distance_bracket`] -- see its doc for why an
/// interval is what this pair admits instead of a formula. Measured by this
/// test on this state: the bracket is `[3.27873837944664820e-2,
/// 3.27873837944665236e-2]`, `4.163336e-17` wide; this port lands
/// `2.558970e-11` from it and the oracle `8.892588e-5`, a factor of
/// `3.475066e6`. The residual is the reference's.
///
/// The bracket is computed at the pose *this port's* forward kinematics puts
/// the two links in, so a pose disagreement would move it rather than the
/// oracle. It cannot: `moveit-diff --cases 10000 --seed 1 --tol-fk 1e-9` on
/// this fixture -- the same seeded state pool case 4697 is drawn from --
/// passes 10,006 of 10,006, so the two sides' link transforms for these
/// states agree five orders below the residual being attributed.
///
/// That does not open a new `doc/upstream-bugs.md` entry: the cause is the
/// one §281 already filed as
/// `distance-callback-default-tolerance-makes-distance-order-dependent`.
/// `cylinder x box` is a blank cell in fcl's narrowphase specialisation table
/// in both orders (`include/fcl/narrowphase/detail/gjk_solver_libccd-inl.h`,
/// the `ShapeDistanceLibccdImpl` primary template), so this pair reaches the
/// generic GJK path with `distanceCallback`'s default `distance_tolerance`
/// of `1e-6` -- an iteration stop threshold, not an error bound. §260.4
/// measured the same pair drifting `4.418002e-4` between that default and a
/// tightened `1e-12`, which is five times the residual this test attributes.
///
/// Note the residual is *inside* §5 Phase 3's `1e-4`, so no sweep row turns
/// red either way; this test exists because a passing row still has an owner,
/// and "passes at 1e-4 with a worst of 8.9e-5" and "agrees to 1e-11" are
/// different claims about this port.
#[test]
fn prbt_link_4_base_link_clearance_brackets_the_separated_residual() {
    /// Case 4697's oracle value, as `--stats-json` recorded it for that state.
    const CASE_4697_ORACLE: f64 = 0.032_876_309_670_400_866;
    /// The same run's value for this port, so this test reproduces the sweep
    /// row rather than measuring some other state that resembles it. Recorded
    /// on the host that ran the sweep, which is not necessarily the host
    /// running this test -- see [`SWEEP_ROW_TOL`].
    const CASE_4697_RUST: f64 = 0.032_787_383_820_056_18;
    /// How far this port's answer may sit from [`CASE_4697_RUST`] before the
    /// row stops being identifiable.
    ///
    /// It is not zero because [`CASE_4697_RUST`] is not portable.
    /// `tools/ci/verify-phase3-collision-sweep.sh` needs docker for the
    /// oracle, so the sweep runs where docker runs, and the pair's minimum
    /// comes out of an iterative descent that does not land on the same `f64`
    /// on a different target. Measured `1.298454e-11` between the recorded
    /// value and an `aarch64-apple-darwin` one -- at identical sources and an
    /// identical `Cargo.lock` (checked out at `f069e050`, the commit that
    /// recorded the pin, which reproduces the divergence rather than being
    /// free of it), and identically in debug and release, so it is the target
    /// and not this port's history that moves it.
    ///
    /// Pinned 77x above that spread, and 8.9e4 times below the `8.9e-5`
    /// residual the row is attributed for -- the distance this assertion
    /// exists to tell apart. The claim that this port is the *near* side of
    /// that residual is [`PORT_TOL`]'s, and that one is bracket-relative and
    /// so holds on both hosts.
    const SWEEP_ROW_TOL: f64 = 1e-9;
    /// How far apart the two bounds may be before the bracket stops being a
    /// third answer. Measured `-1.387779e-17` for link_4's first box and
    /// `4.163336e-17` for its second -- signed, because two closed forms that
    /// are mathematically equal at the optimum round past each other, and an
    /// unsigned assertion would accept a lower bound arbitrarily far *above*
    /// the upper one. Pinned 24x above the larger magnitude, still eight
    /// orders below the residual being attributed.
    const MAX_BRACKET_WIDTH: f64 = 1e-15;
    /// Measured `2.558970e-11` for this port on the host that recorded
    /// [`CASE_4697_RUST`] and `3.857419e-11` on `aarch64-apple-darwin`, so
    /// the margin is 4x on the first and 2.6x on the second. Unlike
    /// [`SWEEP_ROW_TOL`] this bound is against a bracket the test derives
    /// from the model it loaded, so it is the same claim on either host.
    const PORT_TOL: f64 = 1e-10;
    /// Measured `8.892588e-5` for the oracle. Pinned as a *floor*: the claim
    /// is that the reference is far, so the assertion has to fail if it ever
    /// gets close.
    const MIN_ORACLE_ERROR: f64 = 1e-5;

    // The sweep's own state, from the same `--stats-json` tail entry.
    let case_4697: BTreeMap<String, f64> = [
        ("prbt_joint_1", 2.046_935_426_606_275),
        ("prbt_joint_2", 1.951_210_528_619_135_7),
        ("prbt_joint_3", -1.503_936_973_875_760_9),
        ("prbt_joint_4", -1.046_554_922_150_457_3),
        ("prbt_joint_5", 1.268_189_592_042_518_6),
        ("prbt_joint_6", -2.565_574_970_740_266),
    ]
    .into_iter()
    .map(|(name, value)| (name.to_owned(), value))
    .collect();

    let model = build_model("prbt.urdf", "prbt.srdf");
    let mut state = build_state(&model, &case_4697);
    let posed = state.update();

    // Read the geometry out of the model the port itself loaded, not from
    // constants transcribed out of the URDF: a fixture that drifted would
    // then move both this bracket and the port's answer together, and the
    // test would keep passing while measuring a different robot.
    let shapes = |link: &str| -> Vec<WorldConvex> {
        let pose = posed
            .global_link_transform(link)
            .unwrap_or_else(|e| panic!("prbt has a {link} link: {e}"));
        model
            .link_model(link)
            .unwrap_or_else(|e| panic!("prbt has a {link} link model: {e}"))
            .shapes()
            .iter()
            .map(|s| WorldConvex::from_link_shape(&pose, s))
            .collect()
    };
    let base = shapes("prbt_base_link");
    let link_4 = shapes("prbt_link_4");
    assert_eq!(
        base.len(),
        1,
        "prbt_base_link should carry exactly the one <collision> cylinder this bracket is for"
    );
    assert_eq!(
        link_4.len(),
        2,
        "prbt_link_4 should carry exactly the two <collision> boxes §260.4 conflated into one"
    );

    // Every (box, cylinder) combination, so which one wins is measured.
    let mut brackets: Vec<(usize, DistanceBracket)> = Vec::new();
    for (i, box_shape) in link_4.iter().enumerate() {
        let bracket = convex_distance_bracket(box_shape, &base[0]);
        let (lower, upper) = (bracket.lower, bracket.upper);
        assert!(
            (upper - lower).abs() <= MAX_BRACKET_WIDTH,
            "link_4 shape {i}: the bracket is [{lower:.17e}, {upper:.17e}], {:.6e} apart, over \
             the pinned {MAX_BRACKET_WIDTH:.0e} -- it is no longer tight enough to attribute a \
             residual of 8.9e-5 to either side",
            upper - lower
        );
        brackets.push((i, bracket));
    }
    let (winner, bracket) = brackets
        .iter()
        .min_by(|a, b| a.1.upper.total_cmp(&b.1.upper))
        .expect("two shapes were bracketed above");
    let (winner, lower, upper) = (*winner, bracket.lower, bracket.upper);
    let WorldConvex::Box { half, .. } = &link_4[winner] else {
        panic!("prbt_link_4's <collision> shapes are both boxes")
    };
    assert_eq!(
        half.map(|h| (h * 2000.0).round() as i64),
        [90, 60, 120],
        "the nearest of prbt_link_4's two boxes at case 4697 is {half:?} (full size {:?}), not \
         the 0.09 x 0.06 x 0.12 one this test and §260.4's correction are written for",
        half.map(|h| h * 2.0)
    );

    // Which features realise the minimum, measured rather than asserted --
    // this is the reason the pair has no short closed form, and §260.8's
    // successor says so out loud.
    let WorldConvex::Box { centre, axes, .. } = &link_4[winner] else {
        panic!("prbt_link_4's <collision> shapes are both boxes")
    };
    let box_local = {
        let d = bracket.on_a - centre;
        [d.dot(&axes[0]), d.dot(&axes[1]), d.dot(&axes[2])]
    };
    let on_face = [0, 1, 2].map(|k| (box_local[k].abs() - half[k]).abs() <= 1e-12);
    assert_eq!(
        on_face,
        [true, true, false],
        "the box witness {box_local:?} is not in the interior of an edge (half-extents \
         {half:?}); the doc's account of why this pair has no short closed form -- a circle \
         against a line segment -- is derived from it being one"
    );
    let WorldConvex::Cylinder {
        centre,
        axis,
        half_length,
        radius,
    } = &base[0]
    else {
        panic!("prbt_base_link's <collision> shape is a cylinder")
    };
    let d = bracket.on_b - centre;
    let along = d.dot(axis);
    let rho = (d - axis * along).norm();
    assert!(
        (rho - radius).abs() <= 1e-12 && (along.abs() - half_length).abs() <= 1e-12,
        "the cylinder witness is at radius {rho} (r {radius}), axial {along} (h {half_length}) \
         -- not on the rim circle where the two caps meet the side, which is the other half of \
         that account"
    );

    // The port's own answer for the pair, isolated out of the exhaustive
    // minimum-distance search so the number compared below is this pair's and
    // not whichever pair happens to rank first.
    let mut acm = AllowedCollisionMatrix::new();
    for link in model.link_models() {
        acm.set_default_entry(link.name(), true);
    }
    acm.set_entry("prbt_link_4", "prbt_base_link", false);
    let request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };
    let env = floor_env();
    let port = env
        .distance_self(&request, &posed, &[])
        .minimum_distance
        .distance;
    assert!(
        (port - CASE_4697_RUST).abs() <= SWEEP_ROW_TOL,
        "this port now answers {port} for prbt_link_4/prbt_base_link at case 4697, not the \
         {CASE_4697_RUST} the sweep row this test attributes was measured from"
    );

    let port_error = (port - upper).abs().max((port - lower).abs());
    let oracle_error = (CASE_4697_ORACLE - upper)
        .abs()
        .min((CASE_4697_ORACLE - lower).abs());
    assert!(
        port_error <= PORT_TOL,
        "this port is {port_error:.6e} from the bracket [{lower:.17e}, {upper:.17e}], over the \
         pinned {PORT_TOL:.0e} -- the residual would then be this port's"
    );
    assert!(
        CASE_4697_ORACLE > upper || CASE_4697_ORACLE < lower,
        "the oracle's {CASE_4697_ORACLE} now falls inside the bracket [{lower:.17e}, \
         {upper:.17e}] -- the divergence this test attributes is gone"
    );
    assert!(
        oracle_error >= MIN_ORACLE_ERROR,
        "the oracle is only {oracle_error:.6e} from the bracket, under the pinned \
         {MIN_ORACLE_ERROR:.0e} -- this test's claim that the residual is the reference's no \
         longer holds at that margin"
    );
}

/// §297: case 4697 is one member of a 244-case family, and the feature that
/// defines the family is neither the link pair nor the blank specialisation
/// cell that [`prbt_link_4_base_link_clearance_brackets_the_separated_residual`]
/// named.
///
/// That test settled one case on `cylinder x box` being blank in fcl's
/// `ShapeDistanceLibccdImpl` table. Read as a family anchor that is wrong in
/// both directions, and this test is what measures the correction.
///
/// Too narrow, by link pair: prbt's separated tail also holds cases on
/// `prbt_base_link`/`prbt_link_3`, whose minimum is `cylinder x cylinder`, so
/// a `prbt_link_4` anchor reports them as unexplained singletons.
///
/// Too wide, by blank cell: `box x box` is blank in the same table, and it is
/// 8,489 of the 9,611 separated self comparisons -- 88% of the population --
/// with a worst deviation of `4.440892e-16`. A blank cell alone predicts
/// nothing.
///
/// The anchor that does hold is *blank cell AND at least one curved support
/// set*. GJK on two polytopes terminates on an exact vertex/face pair, so
/// `distanceCallback`'s `distance_tolerance` of `1e-6` never binds; give it a
/// cylinder and the support point moves continuously with the search
/// direction, and the stop threshold is what sets the answer. The three cells
/// partition cleanly, measured over the whole separated self population of the
/// `--cases 10000 --seed 1` prbt sweep:
///
/// | winning cell        | fcl        | cases | deviating | worst `|d|`    |
/// |---------------------|------------|-------|-----------|----------------|
/// | `box x box`         | blank      |  8489 |         0 | `4.440892e-16` |
/// | `box x sphere`      | closed     |    73 |         0 | `3.608225e-16` |
/// | `cylinder x sphere` | closed     |   791 |         0 | `5.412337e-16` |
/// | `box x cylinder`    | blank      |   153 |       139 | `8.892585e-5`  |
/// | `cylinder x cylinder` | blank    |   105 |       105 | `2.414557e-5`  |
///
/// "Deviating" is `|oracle - port| > 1e-11`, and that threshold is the data's
/// own: the decades `1e-15` through `1e-11` are empty. 9,367 comparisons sit
/// at `1e-16` or below and 244 sit at `1e-10` or above, with nothing between.
///
/// The closed-form cells are `gjk_solver_libccd-inl.h:679`/`:697` (box/sphere)
/// and `:751`/`:769` (cylinder/sphere); the file's own table above the primary
/// template, captioned "Shape distance algorithms not using libccd", marks the
/// same cells.
///
/// Which side is wrong is then a second question, and the bracket answers it
/// per case rather than by assumption. Over the 244: 191 have the port inside
/// and the oracle outside, 6 have the port outside and the oracle inside, 44
/// have neither offset dominating the other by 100x, and on 3 the bracket is
/// wider than the deviation and cannot speak. So the tail is mostly but not
/// only the reference's -- `crates/cspace-collision/src/parry.rs`'s
/// `parry3d_f64::query::contact` runs its own iterative support-mapping solver
/// on the same blank cells, and case 5649 is where it, not fcl, is the one
/// outside the bracket by `1.379590e-5`.
///
/// The rows below are that measurement in miniature: the eight worst cases by
/// `|d|`, all six port-side cases, and the worst case of each of the three
/// cells that contribute nothing, so a regression that moved a control cell
/// into the family would fail here too.
#[test]
fn prbt_separated_tail_splits_on_the_curved_generic_gjk_cell() {
    /// Which side of the bracket a case's two answers fall on, as measured by
    /// this test rather than asserted from the section.
    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    enum Side {
        /// The port is inside the bracket and the oracle is at least 100x
        /// further from it: the deviation is the reference's.
        Oracle,
        /// The reverse. Six of the 244, and the reason this test does not say
        /// the tail belongs to fcl.
        Port,
        /// Both are at the bracket's own rounding scale.
        Control,
    }
    use Side::{Control, Oracle, Port};

    /// One measured sweep case.
    ///
    /// `oracle` and `port` are the values `moveit-diff --cases 10000 --seed 1
    /// --collision` printed for that case, so this test reproduces sweep rows
    /// rather than measuring nearby states. `joints` is the same run's state,
    /// read back from the oracle's own `random_states` at that seed. `cell`
    /// and `side` are what the assertions below re-derive and compare against.
    struct Row {
        case: u32,
        link_a: &'static str,
        link_b: &'static str,
        oracle: f64,
        port: f64,
        joints: [f64; 6],
        cell: &'static str,
        side: Side,
    }

    const CASES: &[Row] = &[
        // --- the eight worst by |d|, all on a curved blank cell ---
        Row {
            case: 4697,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.032_876_309_670_400_866,
            port: 0.032_787_383_820_056_18,
            joints: [
                2.046_935_426_606_275,
                1.951_210_528_619_135_7,
                -1.503_936_973_875_760_9,
                -1.046_554_922_150_457_3,
                1.268_189_592_042_518_6,
                -2.565_574_970_740_266,
            ],
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 1613,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.031_014_985_469_437_758,
            port: 0.030_951_919_742_005_285,
            joints: [
                0.851_553_251_224_458_1,
                1.235_381_836_361_619_8,
                -2.127_395_165_149_960_3,
                -1.097_632_846_233_565_4,
                2.177_800_636_064_429,
                0.576_180_550_042_335_1,
            ],
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 6083,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.017_935_858_303_653_622,
            port: 0.017_893_930_326_642_243,
            joints: [
                2.250_826_242_100_233,
                -2.322_526_618_663_194,
                1.658_086_572_192_516,
                -0.895_167_816_512_519_6,
                1.590_488_045_436_591,
                -0.994_408_066_262_584_3,
            ],
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 1339,
            link_a: "prbt_base_link",
            link_b: "prbt_link_3",
            oracle: 0.027_654_653_306_817_84,
            port: 0.027_630_507_734_631_677,
            joints: [
                2.025_941_445_825_091,
                -2.228_009_275_569_324_4,
                1.786_804_303_907_509_4,
                -1.143_882_164_284_168,
                -2.386_470_412_323_503,
                0.252_311_130_742_784_8,
            ],
            cell: "cylinder x cylinder",
            side: Oracle,
        },
        Row {
            case: 1198,
            link_a: "prbt_base_link",
            link_b: "prbt_link_3",
            oracle: 0.057_744_433_820_988_14,
            port: 0.057_726_503_283_601_96,
            joints: [
                1.559_323_360_088_970_2,
                -2.219_118_531_242_744_4,
                1.544_796_492_067_538,
                -0.011_022_903_189_556_565,
                0.351_357_564_296_023_8,
                0.450_216_211_135_843_65,
            ],
            cell: "cylinder x cylinder",
            side: Oracle,
        },
        Row {
            case: 7824,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.031_426_263_004_283_76,
            port: 0.031_410_024_507_491_016,
            joints: [
                2.020_553_328_841_840_5,
                2.122_681_268_064_673_6,
                -1.926_729_269_134_998_2,
                0.491_489_375_440_562_26,
                2.000_023_394_251_568,
                0.528_365_709_883_226,
            ],
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 327,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.006_123_920_034_705_172_5,
            port: 0.006_108_567_198_513_176,
            joints: [
                -2.408_657_643_882_679,
                -2.290_219_786_449_587_2,
                1.744_491_585_694_625_7,
                -2.373_420_032_170_722_4,
                -1.561_452_682_248_540_2,
                -1.713_741_576_287_923_6,
            ],
            cell: "box x cylinder",
            side: Oracle,
        },
        // --- every port-side case in the family; 5649 is also 8th by |d| ---
        Row {
            case: 5649,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.029_661_234_823_746_39,
            port: 0.029_675_030_345_831_764,
            joints: [
                -0.691_202_661_610_459_8,
                2.227_705_529_090_673_8,
                -1.715_794_009_803_794_3,
                1.063_135_623_524_794_4,
                1.069_593_839_742_467,
                -1.496_425_563_645_093,
            ],
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 7146,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.066_862_766_710_035_48,
            port: 0.066_865_061_208_439_53,
            joints: [
                1.620_229_256_503_349_4,
                1.825_248_539_225_668_8,
                -1.757_139_708_743_803_2,
                -2.678_158_383_040_688_8,
                -1.664_042_447_073_608_6,
                1.197_706_954_699_382_4,
            ],
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 5665,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.036_106_753_270_615_306,
            port: 0.036_113_410_344_527_835,
            joints: [
                0.256_691_958_972_858_46,
                -2.231_338_063_477_646,
                1.583_508_287_158_794_7,
                -0.585_860_919_736_242_1,
                -1.744_635_748_206_703,
                0.534_896_605_891_529_7,
            ],
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 4397,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.032_042_312_033_412_62,
            port: 0.032_043_493_016_835_05,
            joints: [
                1.286_908_191_446_810_8,
                -2.233_159_342_674_226_4,
                1.692_021_854_342_333_8,
                -1.115_623_662_763_461_5,
                -1.724_614_119_422_603_5,
                2.949_868_448_356_921_3,
            ],
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 5075,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.043_321_675_148_122_64,
            port: 0.043_328_909_571_109_145,
            joints: [
                0.928_507_368_564_801_1,
                2.089_610_862_541_566_6,
                -1.427_336_995_697_300_8,
                -0.761_782_608_197_210_6,
                1.459_447_137_713_022_3,
                -2.771_633_367_949_762,
            ],
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 4871,
            link_a: "prbt_base_link",
            link_b: "prbt_link_4",
            oracle: 0.018_617_510_485_645_288,
            port: 0.018_624_360_733_051_952,
            joints: [
                -1.413_117_475_625_239_4,
                2.065_879_318_967_624_6,
                -1.469_348_923_351_988,
                -1.990_864_856_545_580_4,
                2.674_944_262_325_763_8,
                1.565_216_264_974_437_9,
            ],
            cell: "box x cylinder",
            side: Port,
        },
        // --- the worst case of each cell that contributes no family member ---
        Row {
            case: 7079,
            link_a: "prbt_link_2",
            link_b: "prbt_link_4",
            oracle: 0.107_133_171_705_236_04,
            port: 0.107_133_171_705_235_59,
            joints: [
                -0.513_620_517_740_165_8,
                0.806_634_823_239_837,
                -0.904_574_936_407_897_7,
                2.534_362_967_528_393,
                -0.166_489_225_821_420_56,
                2.893_071_462_002_853_3,
            ],
            cell: "box x box",
            side: Control,
        },
        Row {
            case: 1426,
            link_a: "prbt_link_1",
            link_b: "prbt_link_4",
            oracle: 0.018_469_723_001_560_55,
            port: 0.018_469_723_001_560_19,
            joints: [
                1.909_071_336_998_865_2,
                -1.099_279_022_099_138_2,
                -2.323_425_632_738_694_6,
                -1.598_805_632_491_232_8,
                -1.929_633_104_689_456_6,
                -1.030_606_487_141_931_4,
            ],
            cell: "box x sphere",
            side: Control,
        },
        Row {
            case: 8148,
            link_a: "prbt_base_link",
            link_b: "prbt_link_5",
            oracle: 0.024_945_378_424_847_01,
            port: 0.024_945_378_424_846_468,
            joints: [
                -0.681_765_976_663_856,
                -1.964_314_374_767_602_6,
                1.950_255_394_847_318_2,
                -0.249_711_143_831_703_9,
                -0.471_237_441_636_538_36,
                -1.186_060_955_153_117,
            ],
            cell: "cylinder x sphere",
            side: Control,
        },
    ];

    /// A side verdict needs one offset to dominate the other this far. The
    /// ratios these rows actually reach span `2.545658e2` (case 4871, the
    /// narrowest) to `8.613543e11` (case 1198, the widest), so this threshold
    /// sits below every verdict it admits rather than being fitted to one.
    /// Raising it to `1e4` fails case 5665 at `4.384700e3`, which is what
    /// says the comparison is load-bearing rather than decorative.
    const DOMINANCE: f64 = 100.0;
    /// A `Control` row's two answers must agree this closely -- the three
    /// cells they come from have worst deviations of `4.440892e-16`,
    /// `3.608225e-16` and `5.412337e-16`, so `1e-13` is ~200x above the
    /// largest and still five orders below the family's smallest member.
    const CONTROL_TOL: f64 = 1e-13;
    /// The gap the family threshold sits in. Measured: no separated
    /// comparison of the 9,611 falls between `1e-15` and `1e-11`.
    const FAMILY_FLOOR: f64 = 1e-11;

    let model = build_model("prbt.urdf", "prbt.srdf");
    let mut seen_cells: BTreeSet<&str> = BTreeSet::new();
    let mut sides: BTreeMap<&str, usize> = BTreeMap::new();

    for row in CASES {
        let (case, link_a, link_b) = (&row.case, row.link_a, row.link_b);
        let (oracle, port, joints) = (&row.oracle, &row.port, &row.joints);
        let (want_cell, want_side) = (&row.cell, &row.side);
        let values: BTreeMap<String, f64> = (1..=6)
            .map(|i| (format!("prbt_joint_{i}"), joints[i - 1]))
            .collect();
        let mut state = build_state(&model, &values);
        let posed = state.update();

        // Read the geometry out of the loaded model, never from constants
        // transcribed from the URDF: a fixture edit would then move the
        // bracket and the port's answer together and this test would keep
        // passing about a different robot.
        let shapes = |link: &str| -> Vec<WorldConvex> {
            let pose = posed
                .global_link_transform(link)
                .unwrap_or_else(|e| panic!("case {case}: prbt has a {link} link: {e}"));
            model
                .link_model(link)
                .unwrap_or_else(|e| panic!("case {case}: prbt has a {link} link model: {e}"))
                .shapes()
                .iter()
                .map(|s| WorldConvex::from_link_shape(&pose, s))
                .collect()
        };
        let (a_shapes, b_shapes) = (shapes(link_a), shapes(link_b));

        // Which shape realises the link pair's minimum is measured over every
        // combination, not read off the first `<collision>`: `prbt_link_4`
        // carries two boxes and `prbt_link_5` five shapes of three types.
        let mut best: Option<(DistanceBracket, &str, &str)> = None;
        for a in &a_shapes {
            for b in &b_shapes {
                let bracket = convex_distance_bracket(a, b);
                if best
                    .as_ref()
                    .is_none_or(|(w, _, _)| bracket.upper < w.upper)
                {
                    best = Some((bracket, a.kind(), b.kind()));
                }
            }
        }
        let (bracket, kind_a, kind_b) = best.unwrap_or_else(|| {
            panic!("case {case}: {link_a} and {link_b} both carry at least one shape")
        });

        // The cell is named unordered because fcl's table is symmetric: it
        // specialises `Sphere x Box` and `Box x Sphere` alike, so a verdict
        // that depended on which link the sweep printed first would be about
        // print order, not about geometry.
        let mut kinds = [kind_a, kind_b];
        kinds.sort_unstable();
        let cell = format!("{} x {}", kinds[0], kinds[1]);
        assert_eq!(
            &cell.as_str(),
            want_cell,
            "case {case}: the {link_a}/{link_b} minimum is realised by a {cell} pair, not the \
             recorded {want_cell} -- the cell this case is classified under moved"
        );
        seen_cells.insert(want_cell);

        // Signed both ways: two closed forms that are equal at the optimum
        // round past each other, so an unsigned width would accept a lower
        // bound arbitrarily far above the upper one.
        let (low, high) = (
            bracket.lower.min(bracket.upper),
            bracket.lower.max(bracket.upper),
        );
        let offset = |v: f64| {
            if v < low {
                low - v
            } else if v > high {
                v - high
            } else {
                0.0
            }
        };
        let (port_off, oracle_off) = (offset(*port), offset(*oracle));
        let deviation = (oracle - port).abs();

        match want_side {
            Control => {
                assert!(
                    deviation <= CONTROL_TOL,
                    "case {case} is recorded as a {cell} control and the two sides now differ by \
                     {deviation:.6e}, over the pinned {CONTROL_TOL:.0e} -- a cell that \
                     contributed no family member has started contributing one"
                );
            }
            Oracle | Port => {
                assert!(
                    deviation > FAMILY_FLOOR,
                    "case {case} now differs by only {deviation:.6e}, at or under the \
                     {FAMILY_FLOOR:.0e} floor the family is defined by -- it has left the family \
                     this test classifies"
                );
                // `>` on the widened bracket, so a zero-width bracket cannot
                // decide a side on its own rounding.
                let slack = (high - low).max(f64::MIN_POSITIVE);
                let (near, far) = match want_side {
                    Oracle => (port_off, oracle_off),
                    _ => (oracle_off, port_off),
                };
                assert!(
                    far > DOMINANCE * near.max(slack),
                    "case {case} ({cell}) is recorded as {want_side:?}-side, but the two offsets \
                     from the bracket [{low:.17e}, {high:.17e}] are port {port_off:.6e} and \
                     oracle {oracle_off:.6e} -- {far:.6e} no longer dominates {near:.6e} by \
                     {DOMINANCE:.0}x, so the attribution is not supported"
                );
            }
        }
        *sides
            .entry(match want_side {
                Oracle => "oracle",
                Port => "port",
                Control => "control",
            })
            .or_default() += 1;
    }

    // Floor each population separately. A single total would stay green if
    // the port-side rows vanished into the oracle-side ones, and the six
    // port-side cases are the whole reason this section does not read as
    // "the tail is fcl's".
    assert_eq!(
        sides.get("oracle").copied().unwrap_or_default(),
        7,
        "the seven oracle-side rows are what carry the family's dominant verdict"
    );
    assert_eq!(
        sides.get("port").copied().unwrap_or_default(),
        6,
        "all six port-side cases of the 244 are listed here; losing one silently would turn this \
         test back into the single-sided claim §284 made"
    );
    assert_eq!(
        sides.get("control").copied().unwrap_or_default(),
        3,
        "one control per non-contributing cell, which is what makes the blank-cell anchor \
         falsifiable rather than merely consistent"
    );
    assert_eq!(
        seen_cells,
        BTreeSet::from([
            "box x box",
            "box x cylinder",
            "box x sphere",
            "cylinder x cylinder",
            "cylinder x sphere",
        ]),
        "every cell prbt's separated self population reaches must appear, or the partition this \
         test asserts is only measured on part of its own domain"
    );
}

/// The other half of §260.8's open pair: pr2's separated-branch residual
/// `6.056201e-7`, asked with the *same* instrument as prbt's so the two
/// answers come off one measuring device rather than two.
///
/// §260.3 already settled this residual with a genuine closed form -- the
/// caster wheel's cylinder against the floor box's top face is a
/// cylinder-vs-plane pair, and its separation `0.0792 - 0.074792 = 0.004408`
/// is pose-invariant, which is what
/// `pr2_caster_wheel_floor_clearance_matches_the_closed_form` pins. So the
/// question this test answers is not "does the residual have an owner" but
/// the narrower one §260.8's successor has to answer out loud: *does
/// [`convex_distance_bracket`] reach this pair too?* It does, and the point
/// of asking is that a bracket agreeing with an independently derived
/// constant is a cross-check of both -- the constant comes from subtracting
/// two heights read out of `fixtures/pr2.urdf`, the bracket from two support
/// functions evaluated on the model the port loaded, and they share no
/// arithmetic.
///
/// Measured here, over all eight wheels: the bracket is
/// `[4.40799999991675628e-3, 4.40800000008395587e-3]` at worst,
/// `1.671996e-13` apart, and §260.3's constant is `8.394674e-14` from it.
/// The oracle's published value for this pair (case 6338 of the seed-1
/// sweep, `4.40860562000265372e-3`) is `6.056199e-7` outside -- so the
/// bracket picks the same winner the closed form did, by the same margin.
///
/// The `8.4e-14` is not slop in either instrument, and it is not a free
/// parameter either -- it is predicted. `fixtures/pr2.urdf` writes the caster
/// wheel's `<collision>` roll as `1.57079632679`, a truncated `pi/2`, which
/// leaves the cylinder's axis `4.896528e-12 rad` off horizontal. A cylinder
/// of half-length `0.017` tilted by `t` drops its lowest point by
/// `0.017 * sin(t)`, i.e. `8.324097e-14` -- against a measured gap of
/// `8.325285e-14` on the wheels whose bracket is tightest. So the constant
/// §260.3 derives from the *nominal* geometry is that much above the
/// clearance the fixture actually describes, and part of the `7.766010e-14`
/// worst that `pr2_caster_wheel_floor_clearance_matches_the_closed_form`
/// charges to this backend is the fixture's own rounding.
#[test]
fn pr2_caster_wheel_clearance_bracket_agrees_with_260_3s_closed_form() {
    /// §260.3's closed form for this pair: the wheel cylinder's centre height
    /// less its radius, both out of `fixtures/pr2.urdf`.
    const CLOSED_FORM: f64 = 0.0792 - 0.074_792;
    /// The oracle's own published value on this pair, case 6338 of the
    /// seed-1 10,000-state sweep, as `pr2_caster_wheel_floor_clearance_
    /// matches_the_closed_form` records it. That section and the sweep both
    /// print `4.40860562000265372e-3`, one digit past what an `f64` carries;
    /// this is the same value written so the literal is exactly what the
    /// compiler stores.
    const CASE_6338_ORACLE: f64 = 4.408_605_620_002_654e-3;
    /// Measured `1.671996e-13` apart at worst over the eight wheels -- two
    /// orders wider than prbt's pair in the test above, because this one's
    /// optimum is degenerate (a whole generatrix of the cylinder is equally
    /// near the plane) and the witness direction is correspondingly less
    /// determined. Pinned 60x above the measurement, still six orders below
    /// the residual being attributed.
    const MAX_BRACKET_WIDTH: f64 = 1e-11;
    /// Measured `8.394674e-14` at worst, pinned 119x above it. See the doc
    /// comment for why this is not zero.
    const MAX_CLOSED_FORM_GAP: f64 = 1e-11;
    /// Measured `6.056199e-7` for the oracle, pinned as a floor with a 6x
    /// margin.
    const MIN_ORACLE_ERROR: f64 = 1e-7;

    let model = build_model("pr2.urdf", "pr2.srdf");
    let mut state = build_state(&model, &BTreeMap::new());
    let posed = state.update();

    // The floor of [`floor_env`], as a convex body rather than a `World`
    // object: `WorldConvex::from_link_shape` takes a link's shape, and this
    // side of the pair is not a link. Same numbers as `floor_env_with_top`'s,
    // and the assertion below would fail if they drifted apart, since the
    // constant this bracket is checked against is derived from the top face
    // being the plane `z = 0`.
    let floor = WorldConvex::Box {
        centre: cspace_core::geometry::Vector3::new(0.0, 0.0, -0.05),
        axes: [
            cspace_core::geometry::Vector3::new(1.0, 0.0, 0.0),
            cspace_core::geometry::Vector3::new(0.0, 1.0, 0.0),
            cspace_core::geometry::Vector3::new(0.0, 0.0, 1.0),
        ],
        half: [2.0, 2.0, 0.05],
    };

    // All eight caster wheels, because which one the sweep's argmin lands on
    // is a pose-broken tie (see the closed-form test's doc) and the constant
    // is the same for every one of them.
    let mut wheels = 0;
    for front in ["f", "b"] {
        for side in ["l", "r"] {
            for wheel in ["l", "r"] {
                let link = format!("{front}{side}_caster_{wheel}_wheel_link");
                let pose = posed
                    .global_link_transform(&link)
                    .unwrap_or_else(|e| panic!("pr2 has a {link} link: {e}"));
                let shapes = model
                    .link_model(&link)
                    .unwrap_or_else(|e| panic!("pr2 has a {link} link model: {e}"))
                    .shapes();
                assert_eq!(
                    shapes.len(),
                    1,
                    "{link} should carry exactly the one <collision> cylinder §260.3 derives \
                     its constant from"
                );
                let body = WorldConvex::from_link_shape(&pose, &shapes[0]);
                let bracket = convex_distance_bracket(&body, &floor);
                let (lower, upper) = (bracket.lower, bracket.upper);
                assert!(
                    (upper - lower).abs() <= MAX_BRACKET_WIDTH,
                    "{link}: the bracket is [{lower:.17e}, {upper:.17e}], {:.6e} apart, over the \
                     pinned {MAX_BRACKET_WIDTH:.0e}",
                    upper - lower
                );
                let closed_form_error =
                    (CLOSED_FORM - upper).abs().max((CLOSED_FORM - lower).abs());
                assert!(
                    closed_form_error <= MAX_CLOSED_FORM_GAP,
                    "{link}: §260.3's closed form {CLOSED_FORM} is {closed_form_error:.6e} from \
                     the bracket [{lower:.17e}, {upper:.17e}] -- two instruments that share no \
                     arithmetic disagree, and neither may be published alone"
                );
                let oracle_error = (CASE_6338_ORACLE - upper)
                    .abs()
                    .min((CASE_6338_ORACLE - lower).abs());
                assert!(
                    oracle_error >= MIN_ORACLE_ERROR,
                    "{link}: the oracle's {CASE_6338_ORACLE} is only {oracle_error:.6e} from the \
                     bracket, under the pinned {MIN_ORACLE_ERROR:.0e}"
                );
                wheels += 1;
            }
        }
    }
    assert_eq!(wheels, 8, "sanity: all eight caster wheels were bracketed");
}

/// §297.5's open item: the penetration branch's `249/389` charged to a side.
///
/// §297.4 could say only that no case of the 389 is *classified* wrongly --
/// 254 overlap-certified, 0 separation-certified -- and that the disagreement
/// is therefore about depth values, which the separated branch's instrument
/// cannot reach. [`minkowski_depth_bracket`] is the instrument that can, and
/// this test is the reduced form of the run over the whole population.
///
/// MEASURED over all 389 (probe, not sampled; the run is 3.3s and the reduced
/// table below is what a gate can afford to keep):
///
/// | | |
/// |---|---|
/// | fcl outside the bracket, this port inside (**fcl's**) | 277 |
/// | this port outside, fcl inside (**the port's**) | **0** |
/// | neither dominates | 19 |
/// | bracket not narrow enough to judge | 93 |
///
/// The port lands *inside* the bracket in 332 of 389 and within `1e-9` of it in
/// 388 of 389, worst offset `3.051e-9`; and it names the link pair that
/// actually realises the minimum in 389 of 389. fcl names that pair in 274 of
/// 389, which splits its 277 into 115 where it answered about a pair that is
/// not the deepest and 162 where it answered about the right pair and got the
/// depth wrong.
///
/// A zero in the port column is the result this round was told to distrust, so
/// the table below carries §297.3's six port-side cases as a **control**: they
/// are separated, the same function brackets them, and it must still return
/// `Port` on all six. Six real cases rather than a synthetic perturbation,
/// because §297.3 reached them by an unrelated route -- alternating projection
/// and weak duality -- so agreeing with it is evidence about this instrument
/// and not about a shared construction.
///
/// The other shared construction is forward kinematics: this instrument places
/// its bodies with the port's own `global_link_transform`, so a port-side FK
/// error would move the bracket and the port's answer together and be reported
/// as fcl's. MEASURED against the oracle's `fk` op on 28 of the 389 states,
/// the 8 worst included: 308 link transforms, worst element difference
/// `7.771561e-16`. The deviations judged here are up to `9.554699e-2`, thirteen
/// orders above that, so they are not FK's.
#[test]
fn prbt_penetration_branch_is_bracketed_by_the_minkowski_instrument() {
    /// Which side of the bracket a case's two answers fall on.
    #[derive(Debug, PartialEq, Eq, Clone, Copy, PartialOrd, Ord)]
    enum Side {
        /// The port is inside the bracket and fcl is at least `DOMINANCE`x
        /// further from it.
        Oracle,
        /// The reverse. Zero of the 389 penetration cases; the rows carrying
        /// it are §297.3's separated six, kept so this verdict stays reachable.
        Port,
        /// Both sit off the bracket by comparable amounts. Neither is charged.
        Undominated,
        /// The bracket is not narrow relative to the deviation, so the
        /// instrument declines. Counted apart from `Undominated` because the
        /// two mean opposite things: `Undominated` is a measurement that came
        /// out even, this is a measurement that was not made.
        TooWide,
    }
    use Side::{Oracle, Port, TooWide, Undominated};

    /// One measured case, its two answers, and what this test re-derives.
    struct Row {
        case: u32,
        oracle: f64,
        port: f64,
        joints: [f64; 6],
        /// The link pair realising the minimum signed distance, as measured
        /// here -- not the pair either implementation reported.
        pair: &'static str,
        cell: &'static str,
        side: Side,
    }

    /// How much further from the bracket one side must be before the deviation
    /// is charged to it. §297.3's constant, unchanged, so the two branches'
    /// verdicts are on one scale.
    const DOMINANCE: f64 = 100.0;
    /// The bracket may be at most this fraction of the deviation for a case to
    /// be judged at all. MEASURED: at `0.1` the rule declines 93 of the 389,
    /// every one of them with a deviation under `1e-4` (worst `6.214e-8`), so
    /// it never declines a case the §5 Phase 3 clause would call a failure --
    /// all 249 of those are judged, and all 249 come out fcl's.
    const WIDTH_FRACTION: f64 = 0.1;

    const CASES: &[Row] = &[
        // --- the eight worst of the 389 by |d|; every one of them fcl's ---
        Row {
            case: 3830,
            oracle: -0.0038456786266410636,
            port: -0.09939266469565833,
            joints: [
                1.7222781722564067,
                2.505419442115496,
                -2.3097115610816514,
                -2.3294244395936095,
                0.8788463131179101,
                -2.987396283487771,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 725,
            oracle: -0.009234497554552884,
            port: -0.102189116794398,
            joints: [
                -1.9454267492381483,
                2.3933641974899595,
                -2.268167616163194,
                -1.8894957570981794,
                -0.07743879978066293,
                1.4000124216073755,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 4757,
            oracle: -0.020826775650501995,
            port: -0.10272632473679352,
            joints: [
                -2.654803416922996,
                2.214752444890933,
                -2.3531221254427916,
                -1.0431453608450107,
                1.8113741110644206,
                -0.5110484649104534,
            ],
            pair: "prbt_base_link/prbt_link_5",
            cell: "cylinder x cylinder",
            side: Oracle,
        },
        Row {
            case: 6686,
            oracle: -0.005421222403789236,
            port: -0.08716574933235925,
            joints: [
                -0.27172803652633926,
                -2.298402804827746,
                2.283175140105095,
                1.5475644629089071,
                -1.347651617464805,
                2.735106592000751,
            ],
            pair: "prbt_base_link/prbt_link_5",
            cell: "cylinder x cylinder",
            side: Oracle,
        },
        Row {
            case: 6336,
            oracle: -0.030650864684746003,
            port: -0.11189008097810714,
            joints: [
                -1.0636449835260864,
                -2.467611703758808,
                2.132537724091764,
                1.966310132102044,
                0.67965030239339,
                -0.7187009086833616,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 3129,
            oracle: -0.012982668237207493,
            port: -0.09402662265115407,
            joints: [
                0.07734171296962522,
                -2.3430022245539073,
                2.260625567159336,
                1.9615857587484085,
                0.5713188534899802,
                -3.0656157518656926,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Oracle,
        },
        Row {
            case: 4623,
            oracle: -0.010723432356997001,
            port: -0.08489731327008056,
            joints: [
                1.2003097403543066,
                -2.4164137657102662,
                2.3495206262958237,
                2.4746036623594163,
                -0.47930489706580515,
                1.442286809329698,
            ],
            pair: "prbt_base_link/prbt_link_5",
            cell: "cylinder x sphere",
            side: Oracle,
        },
        Row {
            case: 4554,
            oracle: -0.014250405011507948,
            port: -0.08789679971779304,
            joints: [
                -1.9936694991380444,
                -1.6707517761312238,
                2.3502801645325495,
                1.3932353363365868,
                0.20519160660889924,
                1.7337335389915949,
            ],
            pair: "prbt_base_link/prbt_link_5",
            cell: "cylinder x cylinder",
            side: Oracle,
        },
        // --- the `box x box` cell, all four of the population's members ---
        Row {
            case: 2638,
            oracle: -0.0004052563905101939,
            port: -0.00040525639050958994,
            joints: [
                -0.2987824151047134,
                -1.6637074181552185,
                -2.346929239700362,
                -1.0473004356748703,
                2.6511747667915655,
                0.16181172452364123,
            ],
            pair: "prbt_link_2/prbt_link_4",
            cell: "box x box",
            side: TooWide,
        },
        Row {
            case: 7026,
            oracle: -0.0001514953996990401,
            port: -0.00015149539969852197,
            joints: [
                0.3201407619181462,
                0.22572565762769425,
                2.3466837673243135,
                0.5433167327748891,
                1.8983913801994454,
                1.5207196228157076,
            ],
            pair: "prbt_link_2/prbt_link_4",
            cell: "box x box",
            side: TooWide,
        },
        Row {
            case: 8077,
            oracle: -0.0003168427677867834,
            port: -0.0003168427677862811,
            joints: [
                -2.3960358486207762,
                -0.46672232302328087,
                2.3550565896986986,
                0.9883090018093585,
                1.7313827000785151,
                0.8876005541620215,
            ],
            pair: "prbt_link_2/prbt_link_4",
            cell: "box x box",
            side: TooWide,
        },
        Row {
            case: 9911,
            oracle: -0.0008327939775865795,
            port: -0.0008327939775866211,
            joints: [
                2.8426567855585363,
                -1.0733771091295128,
                2.343919508392364,
                0.508139833306605,
                1.7838413144240626,
                1.4885955115844216,
            ],
            pair: "prbt_link_2/prbt_link_4",
            cell: "box x box",
            side: TooWide,
        },
        // --- undominated: the bracket is narrow and both sides sit off it by similar amounts ---
        Row {
            case: 1852,
            oracle: -0.012080800012405805,
            port: -0.01207601867868618,
            joints: [
                0.060958001009831175,
                2.269546928245206,
                -1.9653517287304623,
                0.27951419041950265,
                -0.3375191933255923,
                -2.7908600581864826,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Undominated,
        },
        Row {
            case: 5376,
            oracle: -0.10460516671311507,
            port: -0.10460412104053335,
            joints: [
                -2.5055790360936427,
                2.1410883202266184,
                -2.2393857683136127,
                -2.018759622064149,
                1.6042309765168001,
                -0.6300273408862855,
            ],
            pair: "prbt_base_link/prbt_link_5",
            cell: "cylinder x cylinder",
            side: Undominated,
        },
        Row {
            case: 5971,
            oracle: -0.11476285219405304,
            port: -0.1147631513038187,
            joints: [
                -0.36914035344963914,
                2.1450310307863125,
                -2.2668569270057604,
                -1.803494964174293,
                -1.7775675809705815,
                -0.7545910273329355,
            ],
            pair: "prbt_base_link/prbt_link_5",
            cell: "cylinder x cylinder",
            side: Undominated,
        },
        // --- too wide to judge: the bracket is not narrow relative to the deviation ---
        Row {
            case: 7479,
            oracle: -0.00024775154773430485,
            port: -0.0002478136839996629,
            joints: [
                2.470122748667551,
                -1.9484797128656228,
                1.795422740195598,
                0.8409419815235304,
                -0.8828339187598506,
                -2.8712737442676164,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: TooWide,
        },
        Row {
            case: 6135,
            oracle: -0.07872105521761923,
            port: -0.0787210604817311,
            joints: [
                1.2108386336182893,
                2.004208353225607,
                -2.099006134764198,
                -2.1508750212715286,
                0.7033091115850212,
                1.1904812781246563,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: TooWide,
        },
        Row {
            case: 5870,
            oracle: -0.08261967372213165,
            port: -0.08261967458872792,
            joints: [
                2.292788147219941,
                -2.1719838074869013,
                2.0748893395012247,
                2.17875304954282,
                -1.4396458826847374,
                0.8832739459206072,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: TooWide,
        },
        Row {
            case: 1507,
            oracle: -0.012011210568296835,
            port: -0.012011210568296835,
            joints: [
                -1.6200171132066752,
                2.193150756333554,
                -2.1375407466550356,
                2.641711294512106,
                -2.2205001295929496,
                3.024812355750678,
            ],
            pair: "prbt_base_link/prbt_link_5",
            cell: "cylinder x sphere",
            side: TooWide,
        },
        // --- the discrimination control: §297.3's six port-side cases, separated branch.
        // A run that stopped being able to report `Port` at all would still pass
        // every row above. ---
        Row {
            case: 5649,
            oracle: 0.02966123482374639,
            port: 0.029675030345831764,
            joints: [
                -0.6912026616104598,
                2.2277055290906738,
                -1.7157940098037943,
                1.0631356235247944,
                1.069593839742467,
                -1.496425563645093,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 7146,
            oracle: 0.06686276671003548,
            port: 0.06686506120843953,
            joints: [
                1.6202292565033494,
                1.8252485392256688,
                -1.7571397087438032,
                -2.6781583830406888,
                -1.6640424470736086,
                1.1977069546993824,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 5665,
            oracle: 0.036106753270615306,
            port: 0.036113410344527835,
            joints: [
                0.25669195897285846,
                -2.231338063477646,
                1.5835082871587947,
                -0.5858609197362421,
                -1.744635748206703,
                0.5348966058915297,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 4397,
            oracle: 0.03204231203341262,
            port: 0.03204349301683505,
            joints: [
                1.2869081914468108,
                -2.2331593426742264,
                1.6920218543423338,
                -1.1156236627634615,
                -1.7246141194226035,
                2.9498684483569213,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 5075,
            oracle: 0.04332167514812264,
            port: 0.043328909571109145,
            joints: [
                0.9285073685648011,
                2.0896108625415666,
                -1.4273369956973008,
                -0.7617826081972106,
                1.4594471377130223,
                -2.771633367949762,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Port,
        },
        Row {
            case: 4871,
            oracle: 0.018617510485645288,
            port: 0.018624360733051952,
            joints: [
                -1.4131174756252394,
                2.0658793189676246,
                -1.469348923351988,
                -1.9908648565455804,
                2.6749442623257638,
                1.5652162649744379,
            ],
            pair: "prbt_base_link/prbt_link_4",
            cell: "box x cylinder",
            side: Port,
        },
    ];

    let model = build_model("prbt.urdf", "prbt.srdf");
    let acm = build_acm("prbt.srdf");

    // The pairs MoveIt would check, read from the SRDF rather than listed:
    // a fixture that stopped disabling a pair has to reach this instrument the
    // same way it reaches the port.
    let with_shapes: Vec<&str> = model
        .link_names()
        .iter()
        .filter(|n| model.link_model(n).is_ok_and(|l| !l.shapes().is_empty()))
        .map(String::as_str)
        .collect();
    let mut checked: Vec<(&str, &str)> = Vec::new();
    for (i, a) in with_shapes.iter().enumerate() {
        for b in &with_shapes[i + 1..] {
            let allowed = acm
                .allowed_collision(a, b)
                .is_some_and(|e| e.kind() == cspace_collision::AllowedCollisionType::Always);
            if !allowed {
                checked.push((a, b));
            }
        }
    }
    // The whole set, not a non-empty check: a bracket taken over a subset of
    // the pairs MoveIt checks is a minimum over the wrong population, and it
    // reports as a *narrow* answer rather than as a missing one. MEASURED from
    // `fixtures/prbt.srdf` -- seven pairs survive its `disable_collisions`.
    let named: BTreeSet<String> = checked.iter().map(|(a, b)| format!("{a}/{b}")).collect();
    assert_eq!(
        named,
        BTreeSet::from(
            [
                "prbt_base_link/prbt_link_3",
                "prbt_base_link/prbt_link_4",
                "prbt_base_link/prbt_link_5",
                "prbt_base_link/prbt_flange",
                "prbt_link_1/prbt_link_4",
                "prbt_link_1/prbt_link_5",
                "prbt_link_2/prbt_link_4",
            ]
            .map(String::from)
        ),
        "the self pairs prbt's SRDF leaves to check moved; every bracket below is a minimum \
         over this set and would silently be a minimum over a different one"
    );

    let mut sides: BTreeMap<Side, usize> = BTreeMap::new();
    let mut seen_cells: BTreeSet<&str> = BTreeSet::new();

    for row in CASES {
        let case = row.case;
        let values: BTreeMap<String, f64> = (1..=6)
            .map(|i| (format!("prbt_joint_{i}"), row.joints[i - 1]))
            .collect();
        let mut state = build_state(&model, &values);
        let posed = state.update();

        let mut bodies: Vec<(WorldConvex, WorldConvex)> = Vec::new();
        let mut owner: Vec<String> = Vec::new();
        for (a, b) in &checked {
            let shapes = |link: &str| -> Vec<WorldConvex> {
                let pose = posed
                    .global_link_transform(link)
                    .unwrap_or_else(|e| panic!("case {case}: prbt has a {link} link: {e}"));
                model
                    .link_model(link)
                    .unwrap_or_else(|e| panic!("case {case}: prbt has a {link} model: {e}"))
                    .shapes()
                    .iter()
                    .map(|s| WorldConvex::from_link_shape(&pose, s))
                    .collect()
            };
            let (sa, sb) = (shapes(a), shapes(b));
            let mut names = [*a, *b];
            names.sort_unstable();
            for x in &sa {
                for y in &sb {
                    owner.push(format!("{}/{}", names[0], names[1]));
                    bodies.push((x.clone(), y.clone()));
                }
            }
        }

        let (lo, hi, win) = min_signed_distance_over(&bodies, TOL);
        assert_eq!(
            owner[win], row.pair,
            "case {case}: the minimum signed distance is realised by {}, not the recorded {} -- \
             the pair this case is classified under moved",
            owner[win], row.pair
        );
        let mut kinds = [bodies[win].0.kind(), bodies[win].1.kind()];
        kinds.sort_unstable();
        let cell = format!("{} x {}", kinds[0], kinds[1]);
        assert_eq!(
            cell, row.cell,
            "case {case}: the minimum is realised by a {cell} pair, not the recorded {}",
            row.cell
        );
        seen_cells.insert(row.cell);

        // Signed both ways, as in the separated branch: two bounds that meet at
        // the optimum can round past each other.
        let (low, high) = (lo.min(hi), lo.max(hi));
        let offset = |v: f64| {
            if v < low {
                low - v
            } else if v > high {
                v - high
            } else {
                0.0
            }
        };
        let (port_off, oracle_off) = (offset(row.port), offset(row.oracle));
        let deviation = (row.oracle - row.port).abs();
        let width = high - low;

        match row.side {
            TooWide => assert!(
                width > WIDTH_FRACTION * deviation,
                "case {case} is recorded as too wide to judge, but the bracket is now \
                 {width:.6e} against a deviation of {deviation:.6e} -- the instrument has \
                 become able to judge it and the recorded verdict is stale, not wrong"
            ),
            Oracle | Port | Undominated => {
                assert!(
                    width <= WIDTH_FRACTION * deviation,
                    "case {case} is recorded as judged, but the bracket is {width:.6e} against \
                     a deviation of {deviation:.6e}, over the {WIDTH_FRACTION} the rule allows \
                     -- the verdict below would rest on a measurement that was not made"
                );
                // `>` on the widened bracket, so a zero-width bracket cannot
                // decide a side on its own rounding.
                let slack = width.max(f64::MIN_POSITIVE);
                let oracle_dominates = oracle_off > DOMINANCE * port_off.max(slack);
                let port_dominates = port_off > DOMINANCE * oracle_off.max(slack);
                let dominance = match row.side {
                    Oracle => oracle_dominates,
                    Port => port_dominates,
                    // Written as a conjunction of the two positives rather than
                    // negated comparisons: a NaN offset then fails here instead
                    // of passing as "neither dominates".
                    _ => !oracle_dominates && !port_dominates,
                };
                assert!(
                    dominance,
                    "case {case} is recorded as {:?} and no longer measures that way: fcl is \
                     {oracle_off:.6e} off the bracket and the port {port_off:.6e}, bracket width \
                     {width:.6e}, {DOMINANCE}x rule",
                    row.side
                );
            }
        }
        *sides.entry(row.side).or_default() += 1;
    }

    // MEASURED counts of this table, one floor per verdict rather than a total:
    // 8 / 6 / 3 / 8. A collapse that emptied one bucket while the others
    // absorbed its rows would leave any total unchanged.
    for (side, least) in [(Oracle, 8usize), (Port, 6), (Undominated, 3), (TooWide, 8)] {
        assert!(
            sides.get(&side).copied().unwrap_or(0) >= least,
            "{side:?} is down to {} rows from {least} -- the instrument has lost a verdict it \
             used to be able to reach, which no total over the other buckets would show",
            sides.get(&side).copied().unwrap_or(0)
        );
    }
    assert_eq!(
        seen_cells,
        BTreeSet::from([
            "box x box",
            "box x cylinder",
            "cylinder x cylinder",
            "cylinder x sphere",
        ]),
        "the cells this table covers moved. All four of the population's cells are here, and \
         `box x box` is here as a control: it is the cell §297.2 used to falsify the \
         blank-specialisation anchor, all four of its members in this population are too \
         narrow a deviation to judge, and if one of them ever became judgeable the cell has \
         started contributing"
    );
}

/// One row of `prbt_self_penetration_389.json` -- see
/// [`load_prbt_self_penetration_389`] for how the fixture was produced.
#[derive(Deserialize)]
struct SelfPenetrationRow {
    case: u32,
    oracle: f64,
    rust: f64,
    joint_values: BTreeMap<String, f64>,
}

/// PORTING-PLAN.md §302.6's fifth residual bullet: §302.3/§302.4's
/// 277/0/19/93-of-389 totals came from a one-off probe that section's own
/// text says was not committed, and
/// [`prbt_penetration_branch_is_bracketed_by_the_minkowski_instrument`]
/// above keeps only a 31-row reduced table -- the number 389 itself was not
/// re-derivable by any gate.
///
/// This fixture is every self-side case, from the same sweep §270's own
/// table row reproduces (`--urdf fixtures/prbt.urdf --srdf
/// fixtures/prbt.srdf --cases 10000 --seed 1 --collision --tol-distance
/// 1e-4 --oracle tools/moveit-oracle/run-oracle.sh`), whose *oracle* value
/// drew the penetration branch -- written by `tools/moveit-diff`'s new
/// `--self-penetration-json` flag, added this round because no existing
/// flag kept more than `DistanceBranchStats`'s worst-eight tail. MEASURED
/// against a fresh `--stats-json` run on this tree right before this
/// fixture was captured: `bool_disagrees` 6854, `self_bool_disagrees` 0,
/// `robot_bool_disagrees` 6854, `penetrating.total` 10389 -- all four match
/// §270's table row for prbt cell-for-cell, so this is the same run. 389
/// rows, matching §297.4's count exactly.
fn load_prbt_self_penetration_389() -> Vec<SelfPenetrationRow> {
    let path = fixture_path("prbt_self_penetration_389.json");
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

/// Re-derives
/// [`prbt_penetration_branch_is_bracketed_by_the_minkowski_instrument`]'s
/// verdict rule over the *whole* 389-case population in
/// [`load_prbt_self_penetration_389`], not the 31 rows that test hand-picks,
/// and asserts the published totals: fcl's 277, the port's 0, undominated
/// 19, too-wide-to-judge 93. The classification is re-derived from
/// (oracle_off, port_off, width, deviation) alone, the same closed rule that
/// test applies per recorded verdict, rather than read off a per-row label
/// this fixture does not carry.
#[test]
fn prbt_penetration_branch_full_389_population_matches_the_published_totals() {
    #[derive(Debug, PartialEq, Eq, Clone, Copy, PartialOrd, Ord)]
    enum Side {
        Oracle,
        Port,
        Undominated,
        TooWide,
    }
    use Side::{Oracle, Port, TooWide, Undominated};

    /// §297.3's constant, unchanged -- see the reduced table's own doc.
    const DOMINANCE: f64 = 100.0;
    /// §302.3's constant, unchanged.
    const WIDTH_FRACTION: f64 = 0.1;

    let model = build_model("prbt.urdf", "prbt.srdf");
    let acm = build_acm("prbt.srdf");

    let with_shapes: Vec<&str> = model
        .link_names()
        .iter()
        .filter(|n| model.link_model(n).is_ok_and(|l| !l.shapes().is_empty()))
        .map(String::as_str)
        .collect();
    let mut checked: Vec<(&str, &str)> = Vec::new();
    for (i, a) in with_shapes.iter().enumerate() {
        for b in &with_shapes[i + 1..] {
            let allowed = acm
                .allowed_collision(a, b)
                .is_some_and(|e| e.kind() == cspace_collision::AllowedCollisionType::Always);
            if !allowed {
                checked.push((a, b));
            }
        }
    }
    assert_eq!(
        checked.len(),
        7,
        "prbt's SRDF-derived candidate self-pair set moved from the 7 pairs the reduced \
         table's own assertion pins -- this population is a minimum over a different set now"
    );

    let rows = load_prbt_self_penetration_389();
    assert_eq!(
        rows.len(),
        389,
        "the fixture's own row count moved from §297.4's 389"
    );

    let mut sides: BTreeMap<Side, usize> = BTreeMap::new();
    for row in &rows {
        let mut state = build_state(&model, &row.joint_values);
        let posed = state.update();

        let mut bodies: Vec<(WorldConvex, WorldConvex)> = Vec::new();
        for (a, b) in &checked {
            let shapes = |link: &str| -> Vec<WorldConvex> {
                let pose = posed
                    .global_link_transform(link)
                    .unwrap_or_else(|e| panic!("case {}: prbt has a {link} link: {e}", row.case));
                model
                    .link_model(link)
                    .unwrap_or_else(|e| panic!("case {}: prbt has a {link} model: {e}", row.case))
                    .shapes()
                    .iter()
                    .map(|s| WorldConvex::from_link_shape(&pose, s))
                    .collect()
            };
            let (sa, sb) = (shapes(a), shapes(b));
            for x in &sa {
                for y in &sb {
                    bodies.push((x.clone(), y.clone()));
                }
            }
        }

        let (lo, hi, _win) = min_signed_distance_over(&bodies, TOL);
        let (low, high) = (lo.min(hi), lo.max(hi));
        let offset = |v: f64| {
            if v < low {
                low - v
            } else if v > high {
                v - high
            } else {
                0.0
            }
        };
        let (port_off, oracle_off) = (offset(row.rust), offset(row.oracle));
        let deviation = (row.oracle - row.rust).abs();
        let width = high - low;

        let side = if width > WIDTH_FRACTION * deviation {
            TooWide
        } else {
            let slack = width.max(f64::MIN_POSITIVE);
            let oracle_dominates = oracle_off > DOMINANCE * port_off.max(slack);
            let port_dominates = port_off > DOMINANCE * oracle_off.max(slack);
            match (oracle_dominates, port_dominates) {
                (true, false) => Oracle,
                (false, true) => Port,
                _ => Undominated,
            }
        };
        *sides.entry(side).or_default() += 1;
    }

    assert_eq!(
        (
            sides.get(&Oracle).copied().unwrap_or(0),
            sides.get(&Port).copied().unwrap_or(0),
            sides.get(&Undominated).copied().unwrap_or(0),
            sides.get(&TooWide).copied().unwrap_or(0),
        ),
        (277, 0, 19, 93),
        "the full 389-case verdict moved from §302.3's published (fcl, port, undominated, \
         too-wide) = (277, 0, 19, 93): got {:?}",
        sides
    );
}

/// PORTING-PLAN.md §302.6's third residual bullet: the 19 undominated cases
/// "could be" both solvers erring by the same amount at the same spot, and
/// that was never opened. It measures each undominated row's
/// `(oracle_off, port_off)` rather than reading the aggregate `Undominated`
/// label back, so it stands even if the published 19 ever moves.
#[test]
fn prbt_penetration_branch_undominated_19_are_all_a_near_margin_oracle_lean() {
    const DOMINANCE: f64 = 100.0;
    const WIDTH_FRACTION: f64 = 0.1;

    let model = build_model("prbt.urdf", "prbt.srdf");
    let acm = build_acm("prbt.srdf");
    let with_shapes: Vec<&str> = model
        .link_names()
        .iter()
        .filter(|n| model.link_model(n).is_ok_and(|l| !l.shapes().is_empty()))
        .map(String::as_str)
        .collect();
    let mut checked: Vec<(&str, &str)> = Vec::new();
    for (i, a) in with_shapes.iter().enumerate() {
        for b in &with_shapes[i + 1..] {
            let allowed = acm
                .allowed_collision(a, b)
                .is_some_and(|e| e.kind() == cspace_collision::AllowedCollisionType::Always);
            if !allowed {
                checked.push((a, b));
            }
        }
    }

    // (case, oracle_off, port_off, oracle_off / width) per undominated row.
    let mut undominated: Vec<(u32, f64, f64, f64)> = Vec::new();
    let rows = load_prbt_self_penetration_389();
    for row in &rows {
        let mut state = build_state(&model, &row.joint_values);
        let posed = state.update();
        let mut bodies: Vec<(WorldConvex, WorldConvex)> = Vec::new();
        for (a, b) in &checked {
            let shapes = |link: &str| -> Vec<WorldConvex> {
                let pose = posed
                    .global_link_transform(link)
                    .unwrap_or_else(|e| panic!("case {}: prbt has a {link} link: {e}", row.case));
                model
                    .link_model(link)
                    .unwrap_or_else(|e| panic!("case {}: prbt has a {link} model: {e}", row.case))
                    .shapes()
                    .iter()
                    .map(|s| WorldConvex::from_link_shape(&pose, s))
                    .collect()
            };
            let (sa, sb) = (shapes(a), shapes(b));
            for x in &sa {
                for y in &sb {
                    bodies.push((x.clone(), y.clone()));
                }
            }
        }
        let (lo, hi, _win) = min_signed_distance_over(&bodies, TOL);
        let (low, high) = (lo.min(hi), lo.max(hi));
        let offset = |v: f64| {
            if v < low {
                low - v
            } else if v > high {
                v - high
            } else {
                0.0
            }
        };
        let (port_off, oracle_off) = (offset(row.rust), offset(row.oracle));
        let deviation = (row.oracle - row.rust).abs();
        let width = high - low;
        if width > WIDTH_FRACTION * deviation {
            continue;
        }
        let slack = width.max(f64::MIN_POSITIVE);
        let oracle_dominates = oracle_off > DOMINANCE * port_off.max(slack);
        let port_dominates = port_off > DOMINANCE * oracle_off.max(slack);
        if !oracle_dominates && !port_dominates {
            undominated.push((row.case, oracle_off, port_off, oracle_off / width));
        }
    }

    assert_eq!(
        undominated.len(),
        19,
        "the undominated count moved from §302.3's published 19: got {undominated:?}"
    );

    // MEASURED: in every one of the 19, port_off is exactly 0.0 -- the
    // port's own value sits *inside* the geometric bracket, not offset from
    // it at all -- while oracle_off is strictly positive and, scaled by the
    // bracket width, sits strictly between 10x and 100x. This is the
    // opposite of "both solvers err by the same amount": these are cases
    // where only the oracle sits outside the bracket, by a margin under the
    // 100x DOMINANCE bar rather than over it.
    for &(case, oracle_off, port_off, ratio) in &undominated {
        assert_eq!(
            port_off, 0.0,
            "case {case}: port_off is nonzero ({port_off:e}) -- the 19 are no longer all \
             a within-bracket port value against an outside-bracket oracle value"
        );
        assert!(
            oracle_off > 0.0,
            "case {case}: oracle_off is zero -- this row should not have been undominated"
        );
        assert!(
            (10.0..100.0).contains(&ratio),
            "case {case}: oracle_off/width = {ratio:.3} left the measured (10x, 100x) band"
        );
    }
}

/// PORTING-PLAN.md §302.6's fourth residual bullet: how many of the 93
/// too-wide-to-judge rows become judgeable if the bracket narrows from the
/// published `TOL = 1e-9` to `1e-12` -- three orders tighter, not measured
/// before. Re-runs the same verdict rule
/// [`prbt_penetration_branch_full_389_population_matches_the_published_totals`]
/// applies, over the whole 389-row population again rather than just the 93,
/// since narrowing the bracket can also move a currently-judged row.
#[test]
fn prbt_penetration_branch_at_a_thousandfold_tighter_tolerance() {
    #[derive(Debug, PartialEq, Eq, Clone, Copy, PartialOrd, Ord)]
    enum Side {
        Oracle,
        Port,
        Undominated,
        TooWide,
    }
    use Side::{Oracle, Port, TooWide, Undominated};

    const DOMINANCE: f64 = 100.0;
    const WIDTH_FRACTION: f64 = 0.1;
    /// Three orders below the published [`TOL`] -- the narrowing this
    /// bullet asks about.
    const TIGHT_TOL: f64 = 1e-12;

    let model = build_model("prbt.urdf", "prbt.srdf");
    let acm = build_acm("prbt.srdf");
    let with_shapes: Vec<&str> = model
        .link_names()
        .iter()
        .filter(|n| model.link_model(n).is_ok_and(|l| !l.shapes().is_empty()))
        .map(String::as_str)
        .collect();
    let mut checked: Vec<(&str, &str)> = Vec::new();
    for (i, a) in with_shapes.iter().enumerate() {
        for b in &with_shapes[i + 1..] {
            let allowed = acm
                .allowed_collision(a, b)
                .is_some_and(|e| e.kind() == cspace_collision::AllowedCollisionType::Always);
            if !allowed {
                checked.push((a, b));
            }
        }
    }

    let rows = load_prbt_self_penetration_389();
    let mut sides: BTreeMap<Side, usize> = BTreeMap::new();
    for row in &rows {
        let mut state = build_state(&model, &row.joint_values);
        let posed = state.update();
        let mut bodies: Vec<(WorldConvex, WorldConvex)> = Vec::new();
        for (a, b) in &checked {
            let shapes = |link: &str| -> Vec<WorldConvex> {
                let pose = posed
                    .global_link_transform(link)
                    .unwrap_or_else(|e| panic!("case {}: prbt has a {link} link: {e}", row.case));
                model
                    .link_model(link)
                    .unwrap_or_else(|e| panic!("case {}: prbt has a {link} model: {e}", row.case))
                    .shapes()
                    .iter()
                    .map(|s| WorldConvex::from_link_shape(&pose, s))
                    .collect()
            };
            let (sa, sb) = (shapes(a), shapes(b));
            for x in &sa {
                for y in &sb {
                    bodies.push((x.clone(), y.clone()));
                }
            }
        }
        let (lo, hi, _win) = min_signed_distance_over(&bodies, TIGHT_TOL);
        let (low, high) = (lo.min(hi), lo.max(hi));
        let offset = |v: f64| {
            if v < low {
                low - v
            } else if v > high {
                v - high
            } else {
                0.0
            }
        };
        let (port_off, oracle_off) = (offset(row.rust), offset(row.oracle));
        let deviation = (row.oracle - row.rust).abs();
        let width = high - low;
        let side = if width > WIDTH_FRACTION * deviation {
            TooWide
        } else {
            let slack = width.max(f64::MIN_POSITIVE);
            let oracle_dominates = oracle_off > DOMINANCE * port_off.max(slack);
            let port_dominates = port_off > DOMINANCE * oracle_off.max(slack);
            match (oracle_dominates, port_dominates) {
                (true, false) => Oracle,
                (false, true) => Port,
                _ => Undominated,
            }
        };
        *sides.entry(side).or_default() += 1;
    }

    // MEASURED: narrowing to TIGHT_TOL resolves 58 of the 93 -- but 57 of
    // those 58 land Undominated, not a clean winner. Only 1 flips to a
    // clear Oracle win. Tightening the bracket mostly reveals rows where
    // oracle and port are close to *each other*, not rows with a hidden
    // clear winner.
    assert_eq!(
        (
            sides.get(&Oracle).copied().unwrap_or(0),
            sides.get(&Port).copied().unwrap_or(0),
            sides.get(&Undominated).copied().unwrap_or(0),
            sides.get(&TooWide).copied().unwrap_or(0),
        ),
        (278, 0, 76, 35),
        "the thousandfold-tighter verdict moved from the measured (fcl, port, undominated, \
         too-wide) = (278, 0, 76, 35): got {:?}",
        sides
    );
}
