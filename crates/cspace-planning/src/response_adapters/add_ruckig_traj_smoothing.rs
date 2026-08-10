// Copyright (c) 2021, PickNik Robotics
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_response_adapter_plugins/src/add_ruckig_traj_smoothing.cpp

//! `default_planning_response_adapters::AddRuckigTrajectorySmoothing`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (cpp:40-95)
//!
//! `rclcpp` — 3 occurrences, all logging, discarded (no logging dependency
//! in this crate): one `RCLCPP_DEBUG` (cpp:70), one `RCLCPP_ERROR` (cpp:87),
//! the `rclcpp::Logger logger_` field (cpp:94) and constructor initializer
//! (cpp:55).
//!
//! `moveit_msgs` — 1 occurrence, computation, ported:
//! `moveit_msgs::msg::MoveItErrorCodes::{SUCCESS, FAILURE}` (cpp:83, cpp:88)
//! as `Ok(())`/[`ResponseAdapterError::Failed`].
//!
//! # Not ported: `if (!res.trajectory)` (cpp:71-79)
//!
//! See [`crate::response::PlanningResponse`]'s "No `Option`" section —
//! `trajectory` is never absent in this port.
//!
//! # Ported computation
//!
//! `smoother_.applySmoothing(*res.trajectory,
//! req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor)`
//! (cpp:81) becomes [`cspace_core::trajectory::ruckig_smoothing::apply_smoothing`]
//! — already ported in `cspace-trajectory`, not re-implemented here — called
//! via [`cspace_core::trajectory::trajectory_tools::apply_ruckig_smoothing`]'s
//! convenience wrapper. Upstream's own `add_ruckig_traj_smoothing.cpp:81`
//! does *not* call that wrapper — it calls `smoother_.applySmoothing`
//! directly on its own long-lived `RuckigSmoothing smoother_` member;
//! `trajectory_tools.cpp:70-76`'s free function `applyRuckigSmoothing`
//! (which constructs its own local `RuckigSmoothing time_param`) is a
//! separate convenience entry point upstream offers other callers, not one
//! this adapter itself uses. This port reuses `apply_ruckig_smoothing` here
//! purely for its own convenience (one call instead of constructing
//! [`cspace_core::trajectory::ruckig_smoothing::SmoothingOptions`] by hand), not
//! to reproduce this specific upstream file's call shape,
//! with [`cspace_core::trajectory::ruckig_smoothing::SmoothingOptions::mitigate_overshoot`]/
//! `overshoot_threshold` left at upstream's own defaults (`false`/`0.01`,
//! `RuckigSmoothing`'s two-argument constructor upstream's
//! `AddRuckigTrajectorySmoothing::smoother_` field default-constructs from,
//! `ruckig_traj_smoothing.hpp`), since neither is a
//! [`crate::request::PlanningRequest`] field this adapter has anything else
//! to source them from. `applySmoothing`'s `bool` return (cpp:81) is ported
//! as `apply_ruckig_smoothing`'s existing `Result<(), cspace_core::error::Error>`
//! (`false` here is always paired with an upstream `RCLCPP_ERROR`, discarded
//! per the audit above — same "logging-only, no separate signal" shape as
//! `Result::Err`'s already-ported message).

use crate::scene::PlanningScene;
use cspace_collision::ParryCollisionEnv;
use cspace_core::trajectory::trajectory_tools::apply_ruckig_smoothing;

use crate::PlanningResponseAdapter;
use crate::error::ResponseAdapterError;
use crate::request::PlanningRequest;
use crate::response::PlanningResponse;

/// Upstream's two defaulted trailing `applySmoothing` arguments. See the
/// module doc's "Ported computation" section for why this port cannot read
/// them from [`crate::request::PlanningRequest`].
const MITIGATE_OVERSHOOT: bool = false;
const OVERSHOOT_THRESHOLD: f64 = 0.01;

/// Adapts the response trajectory to be jerk-constrained and time-optimal
/// via Ruckig.
#[derive(Debug, Default)]
pub struct AddRuckigTrajectorySmoothing;

impl PlanningResponseAdapter for AddRuckigTrajectorySmoothing {
    fn description(&self) -> &'static str {
        "AddRuckigTrajectorySmoothing"
    }

    fn adapt<'m>(
        &self,
        _scene: &mut PlanningScene<'m>,
        _env: &ParryCollisionEnv,
        request: &PlanningRequest,
        response: &mut PlanningResponse<'m>,
    ) -> Result<(), ResponseAdapterError> {
        apply_ruckig_smoothing(
            &mut response.trajectory,
            request.max_velocity_scaling_factor,
            request.max_acceleration_scaling_factor,
            MITIGATE_OVERSHOOT,
            OVERSHOOT_THRESHOLD,
        )
        .map_err(|source| ResponseAdapterError::Failed {
            adapter: self.description(),
            source,
        })
    }
}

#[cfg(test)]
mod tests {
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use cspace_core::state::RobotState;
    use cspace_core::trajectory::RobotTrajectory;
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
    fn smooths_a_two_waypoint_panda_arm_trajectory_and_assigns_nonzero_timing() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.4]).unwrap();

        let mut trajectory = RobotTrajectory::for_group_name(&model, "panda_arm").unwrap();
        let start_state = start.clone();
        trajectory.add_suffix_way_point(start, 0.0).unwrap();
        trajectory.add_suffix_way_point(goal, 0.0).unwrap();
        let mut response = PlanningResponse {
            start_state,
            trajectory,
            planner_id: String::new(),
        };

        assert_eq!(
            AddRuckigTrajectorySmoothing.description(),
            "AddRuckigTrajectorySmoothing"
        );
        AddRuckigTrajectorySmoothing
            .adapt(&mut scene, &env, &request(), &mut response)
            .expect("a two-waypoint panda_arm move must smooth successfully");

        assert!(response.trajectory.duration() > 0.0);
    }
}
