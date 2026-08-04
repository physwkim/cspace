// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_request_adapter_plugins/src/check_for_stacked_constraints.cpp

//! `default_planning_request_adapters::CheckForStackedConstraints`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (cpp:40-96)
//!
//! `rclcpp` — 5 occurrences, all logging, all discarded (this crate has no
//! logging dependency, matching `moveit_trajectory::time_optimal_trajectory_generation`'s
//! precedent — see that module's "Out of scope" section):
//!
//! - `rclcpp/logging.hpp`/`rclcpp/node.hpp` includes (cpp:42-43) — no Rust
//!   equivalent needed.
//! - `rclcpp::Logger logger_` field (cpp:95) and its constructor
//!   initializer (cpp:55) — dropped with the logging calls that used it.
//! - Two `RCLCPP_WARN` calls (cpp:73-77, cpp:82-86) — the entire observable
//!   behavior of `adapt`'s body. See "Computation with nowhere left to go"
//!   below for why porting the *condition* these guard produces no
//!   remaining effect once the log call itself is gone.
//!
//! `moveit_msgs` — 1 occurrence, computation, ported: `moveit_msgs::msg::MoveItErrorCodes::SUCCESS`
//! (cpp:90) — the unconditional return value. Ported as `Ok(())`, matching
//! `crate::error`'s module doc convention (`SUCCESS` is `Ok(())`).
//!
//! # Computation with nowhere left to go
//!
//! `req.path_constraints.position_constraints.size() > 1 || ...
//! orientation_constraints.size() > 1` and the equivalent per-`goal_constraints`-entry
//! check (cpp:71, cpp:80) are real computation — counting
//! [`moveit_constraints::Constraint::Position`]/[`moveit_constraints::Constraint::Orientation`]
//! entries. But in upstream's own `adapt`, that count feeds *only* the two
//! `RCLCPP_WARN` calls above; `adapt` returns `SUCCESS` unconditionally
//! either way (cpp:90), with no field of `req` ever mutated. With the warn
//! calls dropped (no logging dependency), there is no remaining reader of
//! the count anywhere in this function — porting the counting arithmetic
//! with nothing left to consume its result would be dead code, not a
//! faithful port of behavior. [`CheckForStackedConstraints::adapt`] is
//! therefore a documented no-op: it always returns `Ok(())`, exactly
//! matching upstream's unconditional `SUCCESS`, without recomputing a count
//! that has no observer.

use moveit_collision::ParryCollisionEnv;
use moveit_scene::PlanningScene;

use crate::PlanningRequestAdapter;
use crate::error::RequestAdapterError;
use crate::request::PlanningRequest;

/// Checks whether `request` carries more than one path or per-goal-set
/// position/orientation constraint. See the module doc for why this is a
/// no-op in this port: upstream only ever used the count to decide whether
/// to log a warning, and this crate has nothing to route that warning
/// through.
#[derive(Debug, Default)]
pub struct CheckForStackedConstraints;

impl PlanningRequestAdapter for CheckForStackedConstraints {
    fn description(&self) -> &'static str {
        "CheckForStackedConstraints"
    }

    fn adapt<'m>(
        &self,
        _scene: &mut PlanningScene<'m>,
        _env: &ParryCollisionEnv,
        _request: &mut PlanningRequest,
    ) -> Result<(), RequestAdapterError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use moveit_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
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

    fn request_with(goal_constraints: Vec<KinematicConstraintSet>) -> PlanningRequest {
        PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal_constraints,
            path_constraints: None,
            workspace_bounds: Default::default(),
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
        }
    }

    #[test]
    fn never_rejects_a_request_no_matter_how_many_constraints_are_stacked() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut goal_set = KinematicConstraintSet::new();
        for _ in 0..3 {
            goal_set.push(Constraint::Joint(
                JointConstraint::new(&model, "panda_joint1", 0.0, 0.01, 0.01, 1.0).unwrap(),
            ));
        }
        let mut request = request_with(vec![goal_set.clone(), goal_set]);

        let adapter = CheckForStackedConstraints;
        assert_eq!(adapter.description(), "CheckForStackedConstraints");
        assert!(adapter.adapt(&mut scene, &env, &mut request).is_ok());
    }
}
