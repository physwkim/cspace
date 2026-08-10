// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Reconstructs `tools/moveit-diff`'s own captured `visibility_cone`
//! mismatch ("case 104": `pr2 --seed 4 --group right_arm --cases 100
//! --constraints 2000`, oracle self_distance `7.47914550966356367e-2` for
//! `bl_caster_l_wheel_link`, this backend's own `2.08696987934593702e-2`
//! -- see `tools/moveit-diff/src/main.rs`'s
//! `a_real_mismatching_case_touches_exactly_one_link`, whose `joint_values`
//! / `sensor_pose` / `target_pose` / `radius` / `cone_sides` are copied
//! verbatim below) and prints the winning triangle (in
//! `bl_caster_l_wheel_link`'s own local, Z-native frame) plus its cylinder
//! geometry to stdout, as the 11 numbers `tools/mpr-vs-epa/mpr_case104.c`
//! reads on stdin: `p0 p1 p2 radius length`.
//!
//! This is the one reconstruction `parry.rs`'s deviation-6(b) doc and
//! `tools/mpr-vs-epa/mpr_case104.c` both depend on -- see that C file's
//! own header comment for why it does not re-derive the triangle itself.
//! `VisibilityConstraint::cone_mesh`'s vertex/triangle formula
//! (`crates/cspace-constraints/src/visibility.rs`) is reproduced here
//! rather than called, because that crate already depends on this one
//! (`Cargo.toml`) -- the reverse edge would be a cycle; this crate's own
//! `tests/collision_parity.rs` reproduces the identical formula for the
//! same reason, and both copies are anchored to the same captured
//! reference depth by the `assert!` below, so they cannot silently drift
//! apart without one of them failing.
//!
//! This backend's own EPA depth (for comparison against the MPR number
//! `mpr_case104.c` reports) and the winning triangle's cone-vertex indices
//! go to stderr, not stdout, so this example's stdout stays exactly the
//! 11 numbers `mpr_case104.c` expects:
//!
//! ```text
//! cargo run --release --example case104_mpr_input -p cspace-collision \
//!     | tools/mpr-vs-epa/build/mpr_case104
//! ```

use std::collections::BTreeMap;
use std::fs;

use cspace_core::geometry::{Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

const TOUCHED_LINK: &str = "bl_caster_l_wheel_link";
const CAPTURED_REFERENCE_DEPTH: f64 = -2.086_969_879_345_937e-2;
const REFERENCE_TOLERANCE: f64 = 1e-9;

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

fn build_pr2_model() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    let urdf_xml = fs::read_to_string(urdf_path).unwrap_or_else(|e| panic!("read pr2.urdf: {e}"));
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture pr2.urdf must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture pr2.srdf must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("pr2 fixture model must build")
}

/// `VisibilityConstraint::cone_mesh`'s exact vertex/triangle formula --
/// see this file's own module doc for why it is reproduced here rather
/// than called.
fn cone_mesh_world(
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

fn main() {
    let model = build_pr2_model();

    let joint_values: BTreeMap<String, f64> = serde_json::from_str(
        r#"{"bl_caster_l_wheel_joint": -2.451585059798172, "bl_caster_r_wheel_joint": -1.2125751462448606, "bl_caster_rotation_joint": 0.129901095290601, "br_caster_l_wheel_joint": 2.093234081841553, "br_caster_r_wheel_joint": 0.0920718799633682, "br_caster_rotation_joint": 1.156251961941016, "fl_caster_l_wheel_joint": 0.4501360411272022, "fl_caster_r_wheel_joint": -2.331468058637221, "fl_caster_rotation_joint": 2.6978024804506067, "fr_caster_l_wheel_joint": 2.0805852854369835, "fr_caster_r_wheel_joint": -0.07704670772749234, "fr_caster_rotation_joint": 2.1595746971716094, "head_pan_joint": 1.772165230598301, "head_tilt_joint": 0.7787539671446244, "l_elbow_flex_joint": -0.2736173052095341, "l_forearm_roll_joint": 1.0488381119058694, "l_gripper_joint": 0.07618819281458854, "l_gripper_l_finger_joint": 0.2638400489529595, "l_gripper_l_finger_tip_joint": 0.2638400489529595, "l_gripper_motor_screw_joint": 2.685487501470873, "l_gripper_motor_slider_joint": -0.02708936585113407, "l_gripper_r_finger_joint": 0.2638400489529595, "l_gripper_r_finger_tip_joint": 0.2638400489529595, "l_shoulder_lift_joint": 0.7195627860737034, "l_shoulder_pan_joint": 0.8130559688981515, "l_upper_arm_roll_joint": 3.427571022603661, "l_wrist_flex_joint": -1.5513118343194947, "l_wrist_roll_joint": -2.2071143516290372, "laser_tilt_mount_joint": -0.35061450647910364, "r_elbow_flex_joint": -0.21371314669155983, "r_forearm_roll_joint": -0.17202537080433045, "r_gripper_joint": 0.03843543348833919, "r_gripper_l_finger_joint": 0.4854168069222942, "r_gripper_l_finger_tip_joint": 0.4854168069222942, "r_gripper_motor_screw_joint": 0.28511155988088, "r_gripper_motor_slider_joint": 0.023785984655842186, "r_gripper_r_finger_joint": 0.4854168069222942, "r_gripper_r_finger_tip_joint": 0.4854168069222942, "r_shoulder_lift_joint": -0.10111868691151032, "r_shoulder_pan_joint": -1.189628085223248, "r_upper_arm_roll_joint": -2.918286682944745, "r_wrist_flex_joint": -1.3908302708994598, "r_wrist_roll_joint": -0.6736757665340329, "torso_lift_joint": 0.2581471112640574, "torso_lift_motor_screw_joint": -0.8098637376411038, "world_joint/theta": -2.836643659765878, "world_joint/x": 0.0, "world_joint/y": 0.0}"#,
    )
    .expect("captured case-104 joint_values must parse");

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    for (name, &value) in &joint_values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    let posed = state.update();

    // Captured verbatim from case 104's own spec -- both poses' rotation
    // is identity, so only the translations matter.
    let world_to_sensor = Isometry3::from_parts(
        nalgebra::Vector3::new(0.30231483312872937, -0.1912422727165995, 0.0842).into(),
        nalgebra::UnitQuaternion::identity(),
    );
    let world_to_target = Isometry3::from_parts(
        nalgebra::Vector3::new(
            0.30231483312872937,
            -0.1912422727165995,
            0.07919999999999999,
        )
        .into(),
        nalgebra::UnitQuaternion::identity(),
    );
    let target_radius = 0.007960615621068475;
    let cone_sides = 5;

    let link = model
        .link_model(TOUCHED_LINK)
        .unwrap_or_else(|e| panic!("{TOUCHED_LINK}: {e}"));
    let shape = &link.shapes()[0];
    let Shape::Cylinder(cylinder) = &shape.shape else {
        panic!("{TOUCHED_LINK} shape[0] is not a cylinder");
    };
    let cyl_frame = posed
        .global_link_transform(TOUCHED_LINK)
        .unwrap_or_else(|e| panic!("{TOUCHED_LINK}: {e}"))
        * shape.origin_transform;

    let (vertices, triangles) = cone_mesh_world(
        &world_to_sensor,
        &world_to_target,
        target_radius,
        cone_sides,
    );

    let to_cyl = cyl_frame.inverse();
    let local_vertices: Vec<parry3d_f64::math::Vector> = vertices
        .iter()
        .map(|v| {
            let p = to_cyl.transform_point(&nalgebra::Point3::from(*v));
            parry3d_f64::math::Vector::new(p.x, p.y, p.z)
        })
        .collect();

    let parry_cylinder = parry3d_f64::shape::Cylinder::new(cylinder.length * 0.5, cylinder.radius);
    // parry's own canonical Cylinder axis is Y; `to_cyl` above already
    // expresses every point in the cylinder shape's own local frame (Z
    // along its axis, matching `convert_shape`'s `axis_fix`), so this
    // query needs that same Y-onto-Z rotation applied to parry's own
    // query pose. Do NOT apply this to `mpr_case104.c`'s `ccd_cyl_t` --
    // libccd's own cylinder support function is already Z-native (see
    // that file's own header comment).
    let axis_fix: parry3d_f64::math::Pose =
        nalgebra::Isometry3::rotation(nalgebra::Vector3::x() * std::f64::consts::FRAC_PI_2).into();
    let identity: parry3d_f64::math::Pose = nalgebra::Isometry3::identity().into();

    let mut best = f64::INFINITY;
    let mut best_tri = [0u32; 3];
    for tri in &triangles {
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

    assert!(
        (best - CAPTURED_REFERENCE_DEPTH).abs() < REFERENCE_TOLERANCE,
        "reconstructed depth {best} moved away from case 104's own captured reference \
         {CAPTURED_REFERENCE_DEPTH} -- the cone_mesh formula or FK reproduced here has drifted \
         from cspace-constraints' own VisibilityConstraint::cone_mesh; re-sync both copies \
         before trusting anything downstream of this program's stdout"
    );

    eprintln!(
        "case 104: this backend's own EPA depth={best} winning cone-mesh triangle={best_tri:?}"
    );

    let p0 = local_vertices[best_tri[0] as usize];
    let p1 = local_vertices[best_tri[1] as usize];
    let p2 = local_vertices[best_tri[2] as usize];
    println!(
        "{:.17e} {:.17e} {:.17e}\n{:.17e} {:.17e} {:.17e}\n{:.17e} {:.17e} {:.17e}\n{:.17e} {:.17e}",
        p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, cylinder.radius, cylinder.length
    );
}
