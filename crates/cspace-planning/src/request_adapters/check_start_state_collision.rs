// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_request_adapter_plugins/src/check_start_state_collision.cpp

//! `default_planning_request_adapters::CheckStartStateCollision`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (cpp:39-113)
//!
//! `rclcpp` — 4 occurrences, all logging, discarded (no logging dependency
//! in this crate): `rclcpp/logging.hpp`/`node.hpp`/`parameter_value.hpp`
//! includes (cpp:43-45), one `RCLCPP_DEBUG` (cpp:71), the `rclcpp::Logger
//! logger_` field (cpp:108) and constructor initializer (cpp:57).
//!
//! `moveit_msgs` — 1 occurrence, computation, ported:
//! `moveit_msgs::msg::MoveItErrorCodes::{SUCCESS, START_STATE_IN_COLLISION}`
//! (cpp:86, cpp:100) — `Ok(())` and
//! [`RequestAdapterError::StartStateInCollision`] respectively.
//!
//! # Ported computation
//!
//! `planning_scene->checkCollision(creq, cres, start_state)` (cpp:81)
//! becomes [`crate::scene::PlanningScene::check_collision`] against the
//! scene's own current state (this crate carries no separate
//! `req.start_state` field — see the crate doc comment's "Deviation"
//! section). `creq.group_name = req.group_name` (cpp:78) is ported as
//! [`cspace_collision::CollisionRequest::group_name`]. The contact-pair
//! message-building loop (cpp:90-98) is ported as
//! [`crate::scene::PlanningScene::colliding_pairs`], joined into the same
//! `"<n> contact(s) detected : <a> - <b>, ..."` format cpp:93/97 builds.

use crate::scene::PlanningScene;
use cspace_collision::{CollisionRequest, ParryCollisionEnv};

use crate::PlanningRequestAdapter;
use crate::error::RequestAdapterError;
use crate::request::PlanningRequest;

/// Checks whether the scene's current state (`request.group_name` narrows
/// which links a group-scoped check considers) collides.
#[derive(Debug, Default)]
pub struct CheckStartStateCollision;

impl PlanningRequestAdapter for CheckStartStateCollision {
    fn description(&self) -> &'static str {
        "CheckStartStateCollision"
    }

    fn adapt<'m>(
        &self,
        scene: &mut PlanningScene<'m>,
        env: &ParryCollisionEnv,
        request: &mut PlanningRequest,
    ) -> Result<(), RequestAdapterError> {
        let creq = CollisionRequest {
            group_name: Some(request.group_name.clone()),
            ..Default::default()
        };
        if !scene.check_collision(env, &creq).collision {
            return Ok(());
        }

        let contacts = scene.colliding_pairs(env, None);
        let mut detail = format!("{} contact(s) detected : ", contacts.len());
        for (a, b) in contacts.keys() {
            detail.push_str(a);
            detail.push_str(" - ");
            detail.push_str(b);
            detail.push_str(", ");
        }
        Err(RequestAdapterError::StartStateInCollision {
            adapter: self.description(),
            detail,
        })
    }
}

#[cfg(test)]
mod tests {
    use cspace_core::geometry::{Cuboid, Isometry3, Shape};
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use std::fs;
    use std::sync::Arc;

    use super::*;

    fn panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// Like [`panda`], but with panda_link*.stl collision meshes actually
    /// resolved (`panda`'s `MeshSearchPaths::none()` leaves every link with
    /// no collision geometry at all, since panda.urdf's `<collision>`
    /// elements are all `<mesh>` references — silently fine for tests that
    /// never need a real collision, but a test asserting an actual
    /// robot/world collision needs geometry that can collide). Same STL set
    /// `collision_parity.rs`'s `fixture_mesh_search_paths` resolves.
    fn panda_with_collision_meshes() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let mesh_paths = MeshSearchPaths::new([(
            "moveit_resources_panda_description",
            format!("{root}/meshes/panda_description"),
        )]);
        let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_paths)
            .expect("fixture model must build");
        (model, srdf)
    }

    fn request() -> PlanningRequest {
        PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal_constraints: vec![],
            path_constraints: None,
            workspace_bounds: Default::default(),
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
            ..Default::default()
        }
    }

    #[test]
    fn an_empty_world_default_pose_does_not_collide() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let adapter = CheckStartStateCollision;
        assert_eq!(adapter.description(), "CheckStartStateCollision");
        assert!(adapter.adapt(&mut scene, &env, &mut request()).is_ok());
    }

    #[test]
    fn a_box_enclosing_the_robot_is_reported_as_a_collision_with_contact_detail() {
        let (model, srdf) = panda_with_collision_meshes();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene.add_shape(
            "engulfing_box",
            Arc::new(Shape::Cuboid(Cuboid::new(4.0, 4.0, 4.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), Default::default());

        let err = CheckStartStateCollision
            .adapt(&mut scene, &env, &mut request())
            .expect_err("a box enclosing the robot must collide");
        match err {
            RequestAdapterError::StartStateInCollision { adapter, detail } => {
                assert_eq!(adapter, "CheckStartStateCollision");
                assert!(detail.contains("contact(s) detected"), "{detail}");
                assert!(detail.contains("engulfing_box"), "{detail}");
            }
            other => panic!("expected StartStateInCollision, got {other:?}"),
        }
    }
}
