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
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, 0.1).expect("4x4x0.1 are valid, positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, -0.05),
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
