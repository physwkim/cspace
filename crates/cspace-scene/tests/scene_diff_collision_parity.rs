// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! PORTING-PLAN.md §5's third Phase 5 completion condition, as a test:
//! "씬 diff 적용 후 충돌 결과가 오라클과 100% 일치".
//!
//! Each case builds a base [`PlanningScene`], collision-checks it, calls
//! [`PlanningScene::diff`] for a child, applies one scene diff to that child,
//! and collision-checks both the child and the parent again — against the C++
//! oracle's `scene_diff_collision` op (`tools/moveit-oracle/src/oracle.cpp`),
//! which does the same three checks through a real
//! `planning_scene::PlanningScene` and its own `diff()`.
//!
//! The five diff kinds the completion condition names each get a case, and
//! `case_labels` below is what ties a case index to its kind — the request
//! fixture is wire JSON with no room for a label, so the mapping lives here
//! and is asserted to cover every case rather than left as a comment that can
//! drift from the fixture:
//!
//! | case | diff kind                                                      |
//! |------|----------------------------------------------------------------|
//! |  1   | world object added                                             |
//! |  2   | world object removed                                           |
//! |  3   | object attached to a link                                      |
//! |  4   | object detached                                                |
//! |  5   | an ACM entry changed                                           |
//! |  6   | added *and* allowed in one diff                                |
//! |  7   | the empty diff                                                 |
//! |  8   | remove prunes the ACM entry it inherited                        |
//! |  9   | a shape added to an object the parent already has               |
//!
//! Cases 6-9 are not extra kinds; they are what make the other five
//! non-vacuous. Case 6 changes the world without changing any collision
//! number, so it fails if `world_object_ids` is not compared. Case 7 is the
//! control: an empty diff must leave the child reading exactly like the
//! parent, so a harness that reported a difference unconditionally would fail
//! it. Case 8 allows a pair, removes the object, and re-adds it: the child
//! collides again only because `remove_object` pruned the ACM entry it had
//! just been given, which no other case can see.
//!
//! **Parent isolation is asserted, not just the child's answer.** Upstream's
//! diff is copy-on-write over separately-inherited layers, and this port
//! mirrors it with `Layered::Inherited`; a child that mutated its parent is
//! exactly the failure that design exists to prevent. Every case therefore
//! checks the parent *after* the diff too, and requires it to read identically
//! to `parent_before` — on this side and on the oracle's.
//!
//! That assertion is worth different amounts on the two sides, and case 9 is
//! why it is worth anything here. On the oracle it is a live check: a C++
//! child holds a mutable `WorldPtr` and a mutable ACM and could write through
//! either. On this side the parent lives behind an `Arc` for as long as the
//! child exists, so the state, ACM and transform layers *cannot* be reached —
//! the isolation there holds by construction rather than by measurement. The
//! one remaining shared-mutable path is [`cspace_collision::World`]'s
//! `Arc<Object>` copy-on-write (upstream `ensureUnique`), and it is reachable
//! only by mutating an object the parent already owns — which cases 1-8, all
//! of which add and remove whole map entries, never do. Case 9 adds a second
//! shape to the inherited `floor`, so it is the one case whose child must
//! clone before writing.
//!
//! Built on pr2 for the reason [`attached_collision_parity`]'s module doc
//! gives: panda and fanuc build with zero collision geometry on this port (no
//! mesh loader), so a world object placed at a robot link would be invisible
//! on this side for a reason that has nothing to do with the diff layer. Only
//! `robot_collision`/`robot_distance` are compared, for the same reason that
//! file and `collision_parity.rs` give: most of pr2's self-collision surface
//! is mesh geometry this port does not load, so `self_*` disagrees almost
//! everywhere and would swamp the signal. The oracle reports `self_*` anyway —
//! `tools/ci/verify-fixture-replay.sh` compares the whole committed response
//! against the live oracle, so those fields are still pinned against drift,
//! just not against this port.
//!
//! Every object in the fixture is placed to touch only the eight caster wheels
//! (cylinders — primitives this port does load), measured with the oracle's
//! own `collision` op:
//!
//! - `floor`, a 4×4×0.1 box with its top face at z = 0, clears the robot by
//!   0.004408 — the wheels' lowest point, already the ground truth in
//!   `pr2_attached_collision.json`.
//! - `riser`, the same box 0.01 higher, penetrates all eight wheels by
//!   0.005592.
//! - `corner_box`, a 0.1³ box under `fl_caster_l_wheel_link` alone (that wheel
//!   spans y ∈ [0.2566, 0.2906]; its nearest neighbour spans [0.1586, 0.1926]
//!   and the rear casters sit at x = −0.2246), penetrates that one wheel by
//!   0.005592 and nothing else — which is what makes a *single* ACM entry
//!   enough to clear the whole scene in cases 5, 6 and 8.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use cspace_collision::{
    AttachedBodyGeometry, CollisionEnv, CollisionRequest, DistanceRequest, LinkPaddingScale,
    ParryCollisionEnv,
};
use cspace_geometry::{Cuboid, Isometry3, Shape, Sphere};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_scene::{AttachedBody, PlanningScene};
use cspace_srdf::SrdfModel;
use nalgebra::{Matrix3, Translation3, UnitQuaternion};

#[derive(Deserialize)]
struct ShapeSpec {
    #[serde(rename = "type")]
    kind: String,
    size: Option<[f64; 3]>,
    radius: Option<f64>,
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
    touch_links: Vec<String>,
}

/// One diff action, tagged exactly the way the oracle's `applySceneDiff`
/// dispatches on it.
///
/// An enum rather than a struct of `Option` fields so that an action name this
/// test does not implement is a *parse* failure. The alternative — matching on
/// a string and falling through — would silently apply nothing and then
/// compare against ground truth captured for a diff that really was applied,
/// which is a passing test that checked nothing.
#[derive(Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
enum DiffAction {
    AddObject {
        id: String,
        pose: [f64; 16],
        shape: ShapeSpec,
    },
    RemoveObject {
        id: String,
    },
    Attach {
        id: String,
        link_name: String,
        shapes: Vec<ShapeSpec>,
        shape_poses: Vec<[f64; 16]>,
        touch_links: Vec<String>,
    },
    Detach {
        id: String,
    },
    SetAcmEntry {
        first: String,
        second: String,
        allowed: bool,
    },
}

#[derive(Deserialize)]
struct RequestCase {
    id: u32,
    joint_values: BTreeMap<String, f64>,
    objects: Vec<ObjectSpec>,
    attached_bodies: Vec<AttachedBodySpec>,
    diff: Vec<DiffAction>,
}

#[derive(Deserialize)]
struct Summary {
    robot_collision: bool,
    robot_distance: f64,
    world_object_ids: Vec<String>,
    attached_body_ids: Vec<String>,
}

#[derive(Deserialize)]
struct DiffResult {
    parent_before: Summary,
    child: Summary,
    parent_after: Summary,
}

#[derive(Deserialize)]
struct ResponseCase {
    id: u32,
    ok: bool,
    result: DiffResult,
}

/// What this side measures for one scene, in the same four fields the oracle
/// reports.
#[derive(Debug, PartialEq)]
struct Observed {
    robot_collision: bool,
    robot_distance: f64,
    world_object_ids: Vec<String>,
    attached_body_ids: Vec<String>,
}

/// `1e-4`, per PORTING-PLAN.md §5's distance tolerance for Phase 3's
/// completion condition — the same constant `attached_collision_parity.rs`
/// compares `robot_distance` at.
const TOLERANCE: f64 = 1e-4;

/// Case index (1-based, matching the fixture's `id`) to the diff kind it
/// covers. Asserted exhaustive against the fixture below.
fn case_labels() -> BTreeMap<u32, &'static str> {
    BTreeMap::from([
        (1, "world object added"),
        (2, "world object removed"),
        (3, "object attached to a link"),
        (4, "object detached"),
        (5, "ACM entry changed"),
        (6, "added and allowed in one diff"),
        (7, "empty diff"),
        (8, "remove prunes the ACM entry"),
        (9, "shape added to an existing object"),
    ])
}

fn fixture_path(name: &str) -> String {
    format!(
        "{}/tests/fixtures/pr2_scene_diff_collision_{name}.json",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn load<T: serde::de::DeserializeOwned>(name: &str) -> Vec<T> {
    let path = fixture_path(name);
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn build_model() -> RobotModel {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    let urdf_xml = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn srdf() -> SrdfModel {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse")
}

fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Matrix3::new(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rotation))
}

fn shape_from(spec: &ShapeSpec) -> Arc<Shape> {
    match spec.kind.as_str() {
        "box" => {
            let size = spec.size.expect("a box shape carries a size");
            Arc::new(Shape::Cuboid(
                Cuboid::new(size[0], size[1], size[2])
                    .expect("fixture box dimensions must be positive"),
            ))
        }
        "sphere" => Arc::new(Shape::Sphere(
            Sphere::new(spec.radius.expect("a sphere shape carries a radius"))
                .expect("fixture sphere radius must be positive"),
        )),
        other => panic!("this fixture only uses box and sphere shapes, not {other}"),
    }
}

fn attach_into(scene: &mut PlanningScene<'_>, spec: &AttachedBodySpec) {
    assert_eq!(
        spec.shapes.len(),
        spec.shape_poses.len(),
        "attached body {} needs one pose per shape",
        spec.id
    );
    scene
        .attach_new(
            &spec.id,
            &spec.link_name,
            spec.shapes.iter().map(shape_from).collect(),
            spec.shape_poses
                .iter()
                .map(isometry_from_row_major)
                .collect(),
            spec.touch_links
                .iter()
                .cloned()
                .collect::<BTreeSet<String>>(),
            BTreeMap::new(),
        )
        .unwrap_or_else(|e| panic!("attaching {}: {e}", spec.id));
}

fn apply_diff(scene: &mut PlanningScene<'_>, diff: &[DiffAction]) {
    for action in diff {
        match action {
            DiffAction::AddObject { id, pose, shape } => {
                scene.add_shape(id, shape_from(shape), isometry_from_row_major(pose));
            }
            DiffAction::RemoveObject { id } => {
                assert!(
                    scene.remove_object(id),
                    "remove_object({id}) found nothing to remove"
                );
            }
            DiffAction::Attach {
                id,
                link_name,
                shapes,
                shape_poses,
                touch_links,
            } => {
                attach_into(
                    scene,
                    &AttachedBodySpec {
                        id: id.clone(),
                        link_name: link_name.clone(),
                        shapes: shapes
                            .iter()
                            .map(|s| ShapeSpec {
                                kind: s.kind.clone(),
                                size: s.size,
                                radius: s.radius,
                            })
                            .collect(),
                        shape_poses: shape_poses.clone(),
                        touch_links: touch_links.clone(),
                    },
                );
            }
            DiffAction::Detach { id } => {
                scene
                    .detach(id)
                    .unwrap_or_else(|e| panic!("detaching {id}: {e}"));
            }
            DiffAction::SetAcmEntry {
                first,
                second,
                allowed,
            } => {
                scene
                    .allowed_collision_matrix_mut()
                    .set_entry(first, second, *allowed);
            }
        }
    }
}

/// The four fields the oracle's `sceneCollisionSummary` reports, measured on
/// this side.
///
/// `robot_distance` goes through a direct signed-distance
/// [`CollisionEnv::distance_robot`] rather than
/// `PlanningScene::distance_to_collision`, for the reason
/// `attached_collision_parity.rs`'s module doc gives: that convenience method
/// reproduces upstream's `enable_signed_distance: false` default and clamps a
/// real penetration to `0.0`, while the oracle captures the signed value.
fn observe(scene: &mut PlanningScene<'_>) -> Observed {
    let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());
    let world_object_ids = scene.world().object_ids();
    let acm = scene.allowed_collision_matrix().clone();
    let bodies: Vec<AttachedBody> = scene.attached_bodies().cloned().collect();
    let attached_body_ids: Vec<String> = bodies.iter().map(|b| b.id().to_owned()).collect();

    let robot_collision = scene
        .check_robot_collision(&env, &CollisionRequest::default())
        .collision;

    let posed = scene.current_state_mut().update();
    let attached: Vec<AttachedBodyGeometry<'_>> =
        bodies.iter().map(AttachedBody::as_geometry).collect();
    let request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };
    let robot_distance = env
        .distance_robot(&request, &posed, &attached)
        .minimum_distance
        .distance;

    Observed {
        robot_collision,
        robot_distance,
        world_object_ids,
        attached_body_ids,
    }
}

fn assert_matches(observed: &Observed, expected: &Summary, case: u32, label: &str, which: &str) {
    assert_eq!(
        observed.robot_collision, expected.robot_collision,
        "case {case} ({label}) {which}: robot_collision"
    );
    assert!(
        (observed.robot_distance - expected.robot_distance).abs() < TOLERANCE,
        "case {case} ({label}) {which}: robot_distance {} != {} (oracle)",
        observed.robot_distance,
        expected.robot_distance
    );
    assert_eq!(
        observed.world_object_ids, expected.world_object_ids,
        "case {case} ({label}) {which}: world_object_ids"
    );
    assert_eq!(
        observed.attached_body_ids, expected.attached_body_ids,
        "case {case} ({label}) {which}: attached_body_ids"
    );
}

#[test]
fn pr2_scene_diff_collision_matches_the_oracle() {
    let model = build_model();
    let srdf = srdf();
    let requests: Vec<RequestCase> = load("request");
    let responses: Vec<ResponseCase> = load("response");
    let labels = case_labels();

    assert_eq!(
        requests.len(),
        responses.len(),
        "one response per request case"
    );
    assert_eq!(
        requests.iter().map(|r| r.id).collect::<Vec<_>>(),
        labels.keys().copied().collect::<Vec<_>>(),
        "every fixture case must be labelled with the diff kind it covers"
    );

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.id, response.id, "request/response ids line up");
        assert!(
            response.ok,
            "case {}: the oracle reported an error",
            request.id
        );
        let case = request.id;
        let label = labels[&case];
        let expected = &response.result;

        let mut parent = PlanningScene::new(&model, &srdf);
        for (name, &value) in &request.joint_values {
            parent
                .current_state_mut()
                .set_variable_position(name, value)
                .unwrap_or_else(|e| panic!("case {case}: setting {name}: {e}"));
        }
        for object in &request.objects {
            parent.add_shape(
                &object.id,
                shape_from(&object.shape),
                isometry_from_row_major(&object.pose),
            );
        }
        for spec in &request.attached_bodies {
            attach_into(&mut parent, spec);
        }

        let before = observe(&mut parent);
        assert_matches(
            &before,
            &expected.parent_before,
            case,
            label,
            "parent_before",
        );

        let parent = Arc::new(parent);
        let mut child = parent.diff();
        apply_diff(&mut child, &request.diff);
        let child_observed = observe(&mut child);
        assert_matches(&child_observed, &expected.child, case, label, "child");
        drop(child);

        // The child is gone, so the parent is uniquely owned again and can be
        // re-checked. `try_unwrap` failing here would mean the child outlived
        // its own summary, which would make the isolation check below read a
        // scene that is still being diffed against.
        let Ok(mut parent) = Arc::try_unwrap(parent) else {
            panic!("case {case} ({label}): the child scene outlived its own summary");
        };
        let after = observe(&mut parent);
        assert_matches(&after, &expected.parent_after, case, label, "parent_after");
        assert_eq!(
            before, after,
            "case {case} ({label}): the diff mutated its parent"
        );
    }
}
