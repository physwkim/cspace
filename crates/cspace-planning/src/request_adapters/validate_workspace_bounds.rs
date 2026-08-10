// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_request_adapter_plugins/src/validate_workspace_bounds.cpp

//! `default_planning_request_adapters::ValidateWorkspaceBounds`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (cpp:40-103)
//!
//! `rclcpp` — 6 occurrences, all logging or ROS-parameter plumbing,
//! discarded (see [`crate::request_adapters::CheckStartStateBounds`]'s
//! module doc for the same "Deviation": `default_workspace_bounds` is a
//! plain constructor argument here, not a `ParamListener`):
//! `rclcpp/logger.hpp`/`logging.hpp`/`node.hpp` includes (cpp:42-44), the
//! `rclcpp::Node::SharedPtr` parameter of `initialize` (cpp:62), one
//! `RCLCPP_DEBUG` (cpp:75) and one `RCLCPP_WARN` (cpp:84), the
//! `rclcpp::Logger logger_` field (cpp:98) and constructor initializer
//! (cpp:56).
//!
//! `moveit_msgs` — 2 occurrences, computation, ported:
//! `moveit_msgs::msg::WorkspaceParameters` (cpp:76, the type being read and
//! conditionally overwritten) as [`crate::request::WorkspaceBounds`], and
//! `moveit_msgs::msg::MoveItErrorCodes::SUCCESS` (cpp:93) as `Ok(())` — this
//! adapter has no failure path, matching upstream (it only ever fills in a
//! default; it never rejects a caller-specified box, however large or
//! small).

use cspace_collision::ParryCollisionEnv;
use cspace_core::geometry::Vector3;
use cspace_scene::PlanningScene;

use crate::PlanningRequestAdapter;
use crate::error::RequestAdapterError;
use crate::request::{PlanningRequest, WorkspaceBounds};

/// If `request.workspace_bounds` is [unspecified](WorkspaceBounds::is_unspecified)
/// — every corner component below `DBL_EPSILON` in magnitude, which is
/// upstream's own test and is wider than the all-zero `Default` — fills it
/// in with a `default_workspace_bounds`-edged cube centered on the origin.
#[derive(Debug, Clone, Copy)]
pub struct ValidateWorkspaceBounds {
    default_workspace_bounds: f64,
}

impl ValidateWorkspaceBounds {
    /// `default_workspace_bounds`: the edge length of the cube substituted
    /// when `request.workspace_bounds` is unset.
    pub fn new(default_workspace_bounds: f64) -> Self {
        Self {
            default_workspace_bounds,
        }
    }
}

impl PlanningRequestAdapter for ValidateWorkspaceBounds {
    fn description(&self) -> &'static str {
        "ValidateWorkspaceBounds"
    }

    fn adapt<'m>(
        &self,
        _scene: &mut PlanningScene<'m>,
        _env: &ParryCollisionEnv,
        request: &mut PlanningRequest,
    ) -> Result<(), RequestAdapterError> {
        if request.workspace_bounds.is_unspecified() {
            let half = self.default_workspace_bounds / 2.0;
            request.workspace_bounds = WorkspaceBounds {
                min_corner: Vector3::new(-half, -half, -half),
                max_corner: Vector3::new(half, half, half),
            };
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use std::fs;

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
    fn an_unset_box_is_replaced_by_a_centered_cube() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let mut request = request();

        let adapter = ValidateWorkspaceBounds::new(4.0);
        assert_eq!(adapter.description(), "ValidateWorkspaceBounds");
        adapter.adapt(&mut scene, &env, &mut request).unwrap();

        assert_eq!(
            request.workspace_bounds.min_corner,
            Vector3::new(-2.0, -2.0, -2.0)
        );
        assert_eq!(
            request.workspace_bounds.max_corner,
            Vector3::new(2.0, 2.0, 2.0)
        );
    }

    /// The boundary `== WorkspaceBounds::default()` could not see: a corner
    /// that is nonzero but under `DBL_EPSILON`. Upstream's six
    /// `std::abs(v) < epsilon` tests all hold, so it substitutes the cube.
    #[test]
    fn a_sub_epsilon_corner_still_counts_as_unspecified() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let mut request = request();
        request.workspace_bounds.min_corner = Vector3::new(1e-17, 0.0, 0.0);
        assert_ne!(
            request.workspace_bounds,
            WorkspaceBounds::default(),
            "setup must be a box the old equality test would have kept"
        );

        ValidateWorkspaceBounds::new(4.0)
            .adapt(&mut scene, &env, &mut request)
            .unwrap();

        assert_eq!(
            request.workspace_bounds.min_corner,
            Vector3::new(-2.0, -2.0, -2.0)
        );
    }

    /// The other side of the same boundary: `DBL_EPSILON` itself is not
    /// `< DBL_EPSILON`, so this box is specified and survives.
    #[test]
    fn a_corner_at_exactly_epsilon_is_a_specified_box() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let mut request = request();
        request.workspace_bounds.max_corner = Vector3::new(f64::EPSILON, 0.0, 0.0);
        let original = request.workspace_bounds;

        ValidateWorkspaceBounds::new(4.0)
            .adapt(&mut scene, &env, &mut request)
            .unwrap();

        assert_eq!(request.workspace_bounds, original);
    }

    /// `std::abs(NaN) < epsilon` is false upstream, so a NaN corner is a
    /// specified box there and must stay one here.
    #[test]
    fn a_nan_corner_is_not_unspecified() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let mut request = request();
        request.workspace_bounds.min_corner = Vector3::new(f64::NAN, 0.0, 0.0);

        ValidateWorkspaceBounds::new(4.0)
            .adapt(&mut scene, &env, &mut request)
            .unwrap();

        assert!(request.workspace_bounds.min_corner.x.is_nan());
        assert_eq!(request.workspace_bounds.max_corner, Vector3::zeros());
    }

    /// A negative sub-epsilon corner: upstream compares `std::abs(v)`, not
    /// `v`, so the sign cannot make a box specified on its own.
    #[test]
    fn a_negative_sub_epsilon_corner_is_unspecified_too() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let mut request = request();
        request.workspace_bounds.max_corner = Vector3::new(0.0, -1e-20, 0.0);

        ValidateWorkspaceBounds::new(4.0)
            .adapt(&mut scene, &env, &mut request)
            .unwrap();

        assert_eq!(
            request.workspace_bounds.max_corner,
            Vector3::new(2.0, 2.0, 2.0)
        );
    }

    /// A box entirely on the negative side of every axis. Every component
    /// is below `DBL_EPSILON` as a *signed* value, so a test that compared
    /// `v` instead of `std::abs(v)` would call this unspecified and throw
    /// the caller's box away; upstream compares magnitudes and keeps it.
    /// The other specified-box case below has positive `max_corner`
    /// components and so cannot tell the two apart.
    #[test]
    fn an_all_negative_box_is_specified_because_the_test_is_on_magnitudes() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let mut request = request();
        request.workspace_bounds = WorkspaceBounds {
            min_corner: Vector3::new(-5.0, -5.0, -5.0),
            max_corner: Vector3::new(-1.0, -1.0, -1.0),
        };
        let original = request.workspace_bounds;

        ValidateWorkspaceBounds::new(4.0)
            .adapt(&mut scene, &env, &mut request)
            .unwrap();

        assert_eq!(request.workspace_bounds, original);
    }

    #[test]
    fn a_caller_specified_box_is_left_untouched() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let mut request = request();
        request.workspace_bounds = WorkspaceBounds {
            min_corner: Vector3::new(-1.0, -1.0, -1.0),
            max_corner: Vector3::new(5.0, 5.0, 5.0),
        };
        let original = request.workspace_bounds;

        ValidateWorkspaceBounds::new(4.0)
            .adapt(&mut scene, &env, &mut request)
            .unwrap();

        assert_eq!(request.workspace_bounds, original);
    }
}
