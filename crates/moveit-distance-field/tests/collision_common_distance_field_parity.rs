// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity tests against the moveit2 C++ oracle's
//! `collision_object_point_decomposition` and `link_body_decomposition` ops,
//! covering `collision_common_distance_field.rs`.
//!
//! `test_collision_distance_field.cpp` builds a `CollisionEnvDistanceField`
//! in every `TEST_F`'s `SetUp()` (see `collision_distance_field_types.rs`'s
//! module doc), so nothing from it exercises
//! `getBodyDecompositionCacheEntry`/`getCollisionObjectPointDecomposition`
//! in isolation either -- these two oracle ops are this module's only
//! ground truth, same as the types slice.
//!
//! # `link_body_decomposition_matches_the_oracle`
//!
//! This is the central verification ask for this task: the posed sphere
//! centres and radii `BodyDecomposition::from_shapes` +
//! `PosedBodySphereDecomposition` produce for a real robot link
//! (`base_bellow_link`) across a group state, built the same way upstream's
//! `addLinkBodyDecompositions` does but composed here entirely from
//! primitives this port already has (`BodyDecomposition::from_shapes`,
//! `PosedBodySphereDecomposition`, `RobotState::global_link_transform`) --
//! see `collision_common_distance_field.rs`'s module doc for why the actual
//! `DistanceFieldCacheEntry`/`addLinkBodyDecompositions` machinery is not
//! ported this round.
//!
//! ## Why this link, and why no mesh search paths
//!
//! `base_bellow_link` is one of PR2's four links carrying a single `<box>`
//! collision shape at an identity origin -- the others being
//! `head_plate_frame` and the two `*_gripper_motor_accelerometer_link`s.
//! Any of the four would serve, and 17 of PR2's links carry non-mesh
//! collision geometry in total, so nothing in this test rests on the link
//! being unique. Panda's and fanuc's own collision geometry is mesh-only,
//! which is why the fixture is a PR2 link at all.
//!
//! `build_pr2_model` below passes [`moveit_model::MeshSearchPaths::none`]
//! deliberately: this test's subject is a primitive shape, so mesh loading
//! cannot change its result, and passing no search paths keeps it from
//! depending on the mesh pipeline at all. That is a choice, not a
//! workaround -- PR2's 18 collision meshes have been committed under
//! `fixtures/meshes/` since `2db5d10`, and `verify-fixture-provenance.sh`
//! checks them against the vendor source. The assert in
//! [`link_body_decomposition_matches_the_oracle`] is what keeps the choice
//! honest: point the fixture at a mesh-only link and it fails loudly,
//! rather than silently decomposing an empty shape list.
//!
//! The fixture's 8 cases are 6 states drawn by the oracle's own
//! `random_states` op (mimic- and bounds-consistent, matching
//! `fk_parity.rs`'s methodology), plus two boundary cases a random sweep
//! would essentially never produce on its own:
//!
//! - Case 6 is a byte-for-byte duplicate of case 0's joint values -- the
//!   same state queried twice must produce identical output.
//! - Case 7 is case 0 with `world_joint/x` moved by `0.01`, well under the
//!   `0.05` field resolution used to build the `BodyDecomposition` -- a
//!   real pose change smaller than the field's own resolution, which is
//!   exactly the case a resolution-blind or pose-blind cache would answer
//!   wrong without it ever showing up in a single, isolated query. This
//!   test asserts both against the oracle (parity) and, as its own explicit
//!   check below, that this port's two computed poses actually differ --
//!   proving the sub-resolution move was not silently swallowed on this
//!   side either.

use std::fs;
use std::sync::Arc;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_collision::World;
use moveit_distance_field::{
    BodyDecomposition, PosedBodySphereDecomposition, collision_object_point_decomposition,
};
use moveit_geometry::{Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::{Matrix3, Translation3, UnitQuaternion};

/// PORTING-PLAN.md §5 Phase 3's stated distance tolerance, matching
/// `collision_distance_field_types_parity.rs`.
const TOL: f64 = 1e-4;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn read_fixture(file_name: &str) -> String {
    let path = fixture_path(file_name);
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

/// Row-major 4x4, matching the oracle's `toRowMajor4x4`/`fromRowMajor4x4`.
fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Matrix3::new(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rotation))
}

fn build_pr2_model() -> RobotModel {
    let urdf_path = fixture_path("pr2.urdf");
    let srdf_path = fixture_path("pr2.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("pr2.urdf must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("pr2.srdf must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("pr2 model must build")
}

// --- collision_object_point_decomposition ---

#[derive(Deserialize)]
struct ShapeSpecWire {
    #[serde(rename = "type")]
    kind: String,
    radius: Option<f64>,
    length: Option<f64>,
    size: Option<[f64; 3]>,
}

impl ShapeSpecWire {
    fn to_shape(&self) -> Arc<Shape> {
        Arc::new(match self.kind.as_str() {
            "sphere" => Shape::Sphere(
                moveit_geometry::Sphere::new(self.radius.expect("sphere radius")).unwrap(),
            ),
            "cylinder" => Shape::Cylinder(
                moveit_geometry::Cylinder::new(
                    self.radius.expect("cylinder radius"),
                    self.length.expect("cylinder length"),
                )
                .unwrap(),
            ),
            "box" => {
                let size = self.size.expect("box size");
                Shape::Cuboid(moveit_geometry::Cuboid::new(size[0], size[1], size[2]).unwrap())
            }
            other => panic!("unsupported shape type in fixture: {other}"),
        })
    }
}

#[derive(Deserialize)]
struct ObjectWire {
    id: String,
    pose: [f64; 16],
    shapes: Vec<ShapeSpecWire>,
    shape_poses: Vec<[f64; 16]>,
}

#[derive(Deserialize)]
struct CopdRequestCase {
    id: u64,
    resolution: f64,
    object: ObjectWire,
}

#[derive(Deserialize)]
struct CopdResult {
    points: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct CopdResponseCase {
    id: u64,
    result: CopdResult,
}

#[test]
fn collision_object_point_decomposition_matches_the_oracle() {
    let requests: Vec<CopdRequestCase> = serde_json::from_str(&read_fixture(
        "collision_object_point_decomposition_request.json",
    ))
    .expect("parse collision_object_point_decomposition_request.json");
    let responses: Vec<CopdResponseCase> = serde_json::from_str(&read_fixture(
        "collision_object_point_decomposition_response.json",
    ))
    .expect("parse collision_object_point_decomposition_response.json");
    assert_eq!(requests.len(), responses.len());

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.id, response.id, "request/response id mismatch");
        let id = request.id;

        let mut world = World::new();
        let pose = isometry_from_row_major(&request.object.pose);
        let shapes: Vec<Arc<Shape>> = request.object.shapes.iter().map(|s| s.to_shape()).collect();
        let shape_poses: Vec<Isometry3> = request
            .object
            .shape_poses
            .iter()
            .map(isometry_from_row_major)
            .collect();
        world
            .add_to_object(&request.object.id, pose, &shapes, &shape_poses)
            .unwrap_or_else(|| panic!("id {id}: add_to_object rejected the fixture object"));
        let obj = world
            .get_object(&request.object.id)
            .unwrap_or_else(|| panic!("id {id}: object must exist after add_to_object"));

        let decomposition = collision_object_point_decomposition(&obj, request.resolution)
            .unwrap_or_else(|e| panic!("id {id}: collision_object_point_decomposition: {e}"));
        let actual_points = decomposition.collision_points();

        assert_eq!(
            actual_points.len(),
            response.result.points.len(),
            "id {id}: point count mismatch"
        );
        for (actual, expected) in actual_points.iter().zip(&response.result.points) {
            assert_relative_eq!(actual.x, expected[0], epsilon = TOL);
            assert_relative_eq!(actual.y, expected[1], epsilon = TOL);
            assert_relative_eq!(actual.z, expected[2], epsilon = TOL);
        }
    }
}

// --- link_body_decomposition ---

#[derive(Deserialize)]
struct LbdRequestCase {
    joint_values: std::collections::HashMap<String, f64>,
}

#[derive(Deserialize)]
struct LbdRequest {
    link: String,
    resolution: f64,
    padding: f64,
    cases: Vec<LbdRequestCase>,
}

#[derive(Deserialize)]
struct LbdSphereDump {
    radius: f64,
}

#[derive(Deserialize)]
struct LbdResultCase {
    sphere_centers: Vec<[f64; 3]>,
    bounding_sphere_center: [f64; 3],
    bounding_sphere_radius: f64,
}

#[derive(Deserialize)]
struct LbdResult {
    collision_spheres: Vec<LbdSphereDump>,
    cases: Vec<LbdResultCase>,
}

#[derive(Deserialize)]
struct LbdResponseEntry {
    result: LbdResult,
}

fn apply_joint_values(state: &mut RobotState<'_>, values: &std::collections::HashMap<String, f64>) {
    state.set_to_default_values();
    for (name, &value) in values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
    }
}

#[test]
fn link_body_decomposition_matches_the_oracle() {
    let model = build_pr2_model();

    let requests: Vec<LbdRequest> =
        serde_json::from_str(&read_fixture("link_body_decomposition_request.json"))
            .expect("parse link_body_decomposition_request.json");
    let responses: Vec<LbdResponseEntry> =
        serde_json::from_str(&read_fixture("link_body_decomposition_response.json"))
            .expect("parse link_body_decomposition_response.json");
    assert_eq!(requests.len(), responses.len());
    let request = &requests[0];
    let response = &responses[0];

    let link = model
        .link_model(&request.link)
        .unwrap_or_else(|e| panic!("unknown link {}: {e}", request.link));
    let shapes: Vec<Shape> = link.shapes().iter().map(|s| s.shape.clone()).collect();
    let origin_transforms: Vec<Isometry3> =
        link.shapes().iter().map(|s| s.origin_transform).collect();
    // `base_bellow_link` was chosen specifically because it has one non-mesh
    // shape -- an empty shape list here would mean the fixture no longer
    // exercises what this test exists to exercise.
    assert!(
        !shapes.is_empty(),
        "fixture link {} has no collision geometry on this port -- pick a different link \
         (this test builds with MeshSearchPaths::none, so a mesh-only-collision link has \
         no shape here regardless of moveit-model's own STL loading support)",
        request.link
    );

    let body_decomposition = Arc::new(
        BodyDecomposition::from_shapes(
            &shapes,
            &origin_transforms,
            request.resolution,
            request.padding,
        )
        .expect("BodyDecomposition::from_shapes"),
    );

    assert_eq!(
        body_decomposition.collision_spheres().len(),
        response.result.collision_spheres.len(),
        "unposed collision sphere count mismatch"
    );
    for (actual, expected) in body_decomposition
        .sphere_radii()
        .iter()
        .zip(&response.result.collision_spheres)
    {
        assert_relative_eq!(*actual, expected.radius, epsilon = TOL);
    }

    assert_eq!(request.cases.len(), response.result.cases.len());

    let mut computed_sphere_centers: Vec<Vec<[f64; 3]>> = Vec::with_capacity(request.cases.len());

    for (case_index, (case, expected)) in
        request.cases.iter().zip(&response.result.cases).enumerate()
    {
        let mut state = RobotState::new(&model);
        apply_joint_values(&mut state, &case.joint_values);
        let posed = state.update();
        let link_transform = posed
            .global_link_transform(&request.link)
            .unwrap_or_else(|e| panic!("case {case_index}: global_link_transform: {e}"));

        let mut sphere_decomposition =
            PosedBodySphereDecomposition::new(Arc::clone(&body_decomposition));
        sphere_decomposition.update_pose(link_transform);

        assert_eq!(
            sphere_decomposition.sphere_centers().len(),
            expected.sphere_centers.len(),
            "case {case_index}: sphere center count mismatch"
        );
        for (actual, expected_center) in sphere_decomposition
            .sphere_centers()
            .iter()
            .zip(&expected.sphere_centers)
        {
            assert_relative_eq!(actual.x, expected_center[0], epsilon = TOL);
            assert_relative_eq!(actual.y, expected_center[1], epsilon = TOL);
            assert_relative_eq!(actual.z, expected_center[2], epsilon = TOL);
        }

        let bounding_center = sphere_decomposition.bounding_sphere_center();
        assert_relative_eq!(
            bounding_center.x,
            expected.bounding_sphere_center[0],
            epsilon = TOL
        );
        assert_relative_eq!(
            bounding_center.y,
            expected.bounding_sphere_center[1],
            epsilon = TOL
        );
        assert_relative_eq!(
            bounding_center.z,
            expected.bounding_sphere_center[2],
            epsilon = TOL
        );
        assert_relative_eq!(
            sphere_decomposition.bounding_sphere_radius(),
            expected.bounding_sphere_radius,
            epsilon = TOL
        );

        computed_sphere_centers.push(
            sphere_decomposition
                .sphere_centers()
                .iter()
                .map(|c| [c.x, c.y, c.z])
                .collect(),
        );
    }

    // -- Explicit boundary checks, on top of the oracle-parity loop above --

    // Case 6 is case 0's joint values verbatim: this port must reproduce the
    // exact same pose for the exact same state, not merely an oracle-close
    // one.
    assert_eq!(
        computed_sphere_centers[0], computed_sphere_centers[6],
        "the same state queried twice must produce identical sphere centers"
    );

    // Case 7 is case 0 with world_joint/x moved by 0.01, well under the
    // 0.05 field resolution used to build body_decomposition. The move must
    // still be reflected in the posed output -- proving this port's
    // from_shapes/update_pose composition does not silently snap or cache
    // stale poses at sub-resolution deltas.
    assert_ne!(
        computed_sphere_centers[0], computed_sphere_centers[7],
        "a real, sub-resolution pose change must still change the posed sphere centers"
    );
    let dx = computed_sphere_centers[7][0][0] - computed_sphere_centers[0][0][0];
    assert_relative_eq!(dx, 0.01, epsilon = TOL);
}
