// Copyright (c) 2026, moveit-rs contributors
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
//! `moveit_model::LinkModel` now loads `<mesh>` collision geometry (STL,
//! resolved through [`MeshSearchPaths`] -- see that type and
//! `moveit-geometry`'s `stl` module), so panda and fanuc (whose collision
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

use std::collections::BTreeMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, DistanceResultsData,
    LinkPaddingScale, ParryCollisionEnv, World,
};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

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
    posed: &moveit_state::Posed<'_, '_>,
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
    posed: &moveit_state::Posed<'_, '_>,
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

/// Replicates `moveit-constraints`' `VisibilityConstraint::cone_mesh` exact
/// vertex/triangle formula (see that method's own doc comment: vertex `0`
/// sensor origin, vertex `1` target center, vertices `2..cone_sides+2` the
/// disc rim) so this crate can drive the same mesh through its own
/// `parry3d_f64::query::contact` without a dependency on
/// `moveit-constraints` -- that crate already depends on this one
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
/// `moveit-constraints`/`tools/moveit-diff`. Returns the winning triangle's
/// depth and indices, plus the target-center vertex (mesh vertex `1`)
/// expressed in the cylinder's own local frame, for the caller to check it
/// landed at the local origin.
fn deepest_cone_triangle_vs_cylinder(
    cyl_frame: &Isometry3,
    cylinder: &moveit_geometry::Cylinder,
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
        let axis = flange.rotation * moveit_geometry::Vector3::new(0.0, 0.0, 1.0);

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

/// One convex collision shape, already placed in world coordinates, reduced
/// to the two closed-form primitives [`convex_distance_bracket`] needs.
///
/// Deliberately not a general shape type, and deliberately only the two
/// variants its callers reach: the bracket below is a *proof* only for convex
/// bodies whose support function and point projection are exact. A mesh has
/// both, but only up to its own triangulation, and calling that a third
/// answer would be circular. Every other `Shape` -- sphere included, though
/// it would be exact -- panics in [`WorldConvex::from_link_shape`] rather
/// than being added unused, so a new caller has to state which shapes it
/// means.
enum WorldConvex {
    Box {
        centre: moveit_geometry::Vector3,
        /// World directions of the box's own three axes, unit length.
        axes: [moveit_geometry::Vector3; 3],
        /// Half-extents along `axes`, i.e. `Cuboid::size` halved.
        half: [f64; 3],
    },
    Cylinder {
        centre: moveit_geometry::Vector3,
        /// World direction of the cylinder's local `+z`, unit length.
        axis: moveit_geometry::Vector3,
        half_length: f64,
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
    fn from_link_shape(link_pose: &Isometry3, link_shape: &moveit_model::LinkShape) -> Self {
        let pose = link_pose * link_shape.origin_transform;
        let centre = pose.translation.vector;
        let dir = |x, y, z| pose.rotation * moveit_geometry::Vector3::new(x, y, z);
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
            other => panic!(
                "convex_distance_bracket has no exact support function for {other:?}; the \
                 bracket it produces would not be a proof"
            ),
        }
    }

    /// `max_{x in S} x . n`, exact for both variants.
    fn support_max(&self, n: &moveit_geometry::Vector3) -> f64 {
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
        }
    }

    /// `min_{x in S} x . n`, by the same closed form on `-n`.
    fn support_min(&self, n: &moveit_geometry::Vector3) -> f64 {
        -self.support_max(&(-n))
    }

    /// The point of `S` nearest `p` -- exact for both variants (clamp in the
    /// box's own frame; clamp radially and axially in the cylinder's).
    fn project(&self, p: &moveit_geometry::Vector3) -> moveit_geometry::Vector3 {
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
        }
    }

    fn centre(&self) -> moveit_geometry::Vector3 {
        match self {
            Self::Box { centre, .. } | Self::Cylinder { centre, .. } => *centre,
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
    on_a: moveit_geometry::Vector3,
    /// The witness point on the second body, in world coordinates.
    on_b: moveit_geometry::Vector3,
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
    /// row rather than measuring some other state that resembles it.
    const CASE_4697_RUST: f64 = 0.032_787_383_820_056_18;
    /// How far apart the two bounds may be before the bracket stops being a
    /// third answer. Measured `-1.387779e-17` for link_4's first box and
    /// `4.163336e-17` for its second -- signed, because two closed forms that
    /// are mathematically equal at the optimum round past each other, and an
    /// unsigned assertion would accept a lower bound arbitrarily far *above*
    /// the upper one. Pinned 24x above the larger magnitude, still eight
    /// orders below the residual being attributed.
    const MAX_BRACKET_WIDTH: f64 = 1e-15;
    /// Measured `2.558970e-11` for this port, with a 4x margin.
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
        (port - CASE_4697_RUST).abs() <= 1e-15,
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
        centre: moveit_geometry::Vector3::new(0.0, 0.0, -0.05),
        axes: [
            moveit_geometry::Vector3::new(1.0, 0.0, 0.0),
            moveit_geometry::Vector3::new(0.0, 1.0, 0.0),
            moveit_geometry::Vector3::new(0.0, 0.0, 1.0),
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
