// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_response_adapter_plugins/src/validate_path.cpp

//! `default_planning_response_adapters::ValidateSolution`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (cpp:39-153)
//!
//! `rclcpp` — 7 occurrences, all logging or a `Publisher` construction,
//! discarded: `rclcpp/`-namespaced includes and the `rclcpp::Node::SharedPtr`
//! parameter of `initialize` (cpp:62), one `RCLCPP_DEBUG` (cpp:85), one
//! `RCLCPP_ERROR` (cpp:88), one `RCLCPP_ERROR_STREAM` (cpp:116), one
//! `RCLCPP_ERROR` (cpp:144), the `rclcpp::Logger logger_` field (cpp:151)
//! and constructor initializer (cpp:56).
//!
//! `visualization_msgs`/contact-publishing — the entire
//! `contacts_publisher_`/`display_contacts_topic` mechanism (cpp:62-73,
//! cpp:107-146) is D1 (rviz `MarkerArray` publishing) and not ported, same
//! exclusion as `display_motion_path.cpp` (see the crate doc comment). With
//! it gone, the verbose re-check loop that exists only to *build* the
//! markers (`isStateValid(..., true)`, `checkCollision`,
//! `getCollisionMarkersFromContacts`, cpp:121-142) has no remaining
//! consumer and is not ported either — it recomputes no information
//! [`crate::scene::PlanningScene::is_path_valid`]'s own `invalid_waypoints`
//! did not already report.
//!
//! `moveit_msgs` — 1 occurrence, computation, ported:
//! `moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN` (cpp:89, cpp:104)
//! as [`ResponseAdapterError::InvalidMotionPlan`]; the `SUCCESS` path is
//! implicit (`Ok(())`, cpp:81-148's fallthrough when `isPathValid` returns
//! `true`).
//!
//! # Not ported: `if (!res.trajectory)` (cpp:86-91)
//!
//! See [`crate::response::PlanningResponse`]'s own doc comment's "No
//! `Option`" section: a [`crate::response::PlanningResponse`] in this crate
//! is only ever constructed once a planner has already succeeded, so
//! `trajectory` is never absent here.
//!
//! # Ported computation
//!
//! `planning_scene->isPathValid(*res.trajectory, req.path_constraints,
//! req.group_name, false, &indices)` (cpp:101) becomes
//! [`crate::scene::PlanningScene::is_path_valid`], fed the response
//! trajectory's waypoints (cloned out of
//! [`cspace_core::trajectory::RobotTrajectory`] — `is_path_valid` takes
//! `&[RobotState]`, not a `RobotTrajectory`) and `request.path_constraints`.
//! `request.goal_constraints` (this crate's `Vec<KinematicConstraintSet>`,
//! see the crate doc comment) is passed through as `is_path_valid`'s own
//! `goal_constraints` parameter — upstream's overload used here
//! (`isPathValid(trajectory, path_constraints, group, ...)`, cpp:101) does
//! not itself check goal constraints, but `is_path_valid` accepting them
//! directly is this port's one call, not two, and an empty
//! `goal_constraints` (this adapter's actual upstream-matching case, since
//! upstream's overload never passes any) makes the goal check a no-op by
//! `is_path_valid`'s own contract ("empty means no goal check").

use crate::scene::PlanningScene;
use cspace_collision::{CollisionRequest, ParryCollisionEnv};

use crate::PlanningResponseAdapter;
use crate::error::ResponseAdapterError;
use crate::request::PlanningRequest;
use crate::response::PlanningResponse;

/// Checks the response trajectory for validity (collision avoidance and
/// path-constraint satisfaction). See the module doc for why this port
/// carries no contact-marker publishing path.
#[derive(Debug, Default)]
pub struct ValidateSolution;

impl PlanningResponseAdapter for ValidateSolution {
    fn description(&self) -> &'static str {
        "ValidateSolution"
    }

    fn adapt<'m>(
        &self,
        scene: &mut PlanningScene<'m>,
        env: &ParryCollisionEnv,
        request: &PlanningRequest,
        response: &mut PlanningResponse<'m>,
    ) -> Result<(), ResponseAdapterError> {
        let waypoints: Vec<_> = (0..response.trajectory.way_point_count())
            .map(|i| response.trajectory.way_point(i).unwrap().clone())
            .collect();

        let validity = scene.is_path_valid(
            env,
            &CollisionRequest::default(),
            &waypoints,
            request.path_constraints.as_ref(),
            &request.goal_constraints,
        );
        if validity.valid {
            Ok(())
        } else {
            Err(ResponseAdapterError::InvalidMotionPlan {
                adapter: self.description(),
                invalid_waypoints: validity.invalid_waypoints,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use cspace_core::geometry::{Cuboid, Isometry3, Shape};
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use cspace_core::trajectory::RobotTrajectory;
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
    fn a_single_waypoint_empty_world_trajectory_is_valid() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut trajectory = RobotTrajectory::for_group(&model, None);
        trajectory
            .add_suffix_way_point(scene.current_state().clone(), 0.0)
            .unwrap();
        let mut response = PlanningResponse {
            start_state: scene.current_state().clone(),
            trajectory,
            planner_id: String::new(),
        };

        assert_eq!(ValidateSolution.description(), "ValidateSolution");
        assert!(
            ValidateSolution
                .adapt(&mut scene, &env, &request(), &mut response)
                .is_ok()
        );
    }

    #[test]
    fn a_waypoint_inside_an_obstacle_is_reported_as_invalid_by_index() {
        let (model, srdf) = panda_with_collision_meshes();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene.add_shape(
            "engulfing_box",
            Arc::new(Shape::Cuboid(Cuboid::new(4.0, 4.0, 4.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), Default::default());

        let mut trajectory = RobotTrajectory::for_group(&model, None);
        trajectory
            .add_suffix_way_point(scene.current_state().clone(), 0.0)
            .unwrap();
        let mut response = PlanningResponse {
            start_state: scene.current_state().clone(),
            trajectory,
            planner_id: String::new(),
        };

        let err = ValidateSolution
            .adapt(&mut scene, &env, &request(), &mut response)
            .expect_err("a waypoint inside an obstacle must be invalid");
        match err {
            ResponseAdapterError::InvalidMotionPlan {
                adapter,
                invalid_waypoints,
            } => {
                assert_eq!(adapter, "ValidateSolution");
                assert_eq!(invalid_waypoints, vec![0]);
            }
            other => panic!("expected InvalidMotionPlan, got {other:?}"),
        }
    }
}
