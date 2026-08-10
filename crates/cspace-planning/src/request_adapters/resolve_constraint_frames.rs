// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2019, Bielefeld University
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_request_adapter_plugins/src/resolve_constraint_frames.cpp
//   moveit_core/kinematic_constraints/src/utils.cpp (resolveConstraintFrames, cpp:623-676)

//! `default_planning_request_adapters::ResolveConstraintFrames`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (adapter cpp:41-83)
//!
//! `rclcpp` — 1 occurrence, logging, discarded: one `RCLCPP_DEBUG` (cpp:65,
//! via the `moveit::utils::logger` helper — no separate `rclcpp::Logger`
//! field here, unlike this crate's other four request adapters, since the
//! upstream class stores one but the adapter body only ever calls it once).
//!
//! `moveit_msgs` — 1 occurrence, computation, ported:
//! `moveit_msgs::msg::MoveItErrorCodes::SUCCESS` (cpp:73) — `Ok(())`,
//! unconditionally, matching upstream (this adapter has no failure path:
//! `kinematic_constraints::resolveConstraintFrames`'s own `bool` return can
//! be `false`, but the adapter's `adapt` never inspects it before returning
//! `SUCCESS` regardless — confirmed from cpp:66-73, the return value of
//! `resolveConstraintFrames` is discarded).
//!
//! # Why this port is a structural no-op, not a shortcut
//!
//! Upstream's `resolveConstraintFrames(state, constraints)`
//! (`kinematic_constraints/src/utils.cpp:623-676`) mutates a raw
//! `moveit_msgs::msg::Constraints`: for each `PositionConstraint`/
//! `OrientationConstraint`, it looks up `c.link_name` via
//! `state.getFrameInfo(c.link_name, robot_link, frame_found)` — a lookup
//! that accepts an attached-body/subframe name, not just a plain robot link
//! — and, if `c.link_name != robot_link->getName()` (the name resolved to a
//! subframe), rewrites `c.link_name` to the real robot link and folds the
//! subframe/link offset into `target_point_offset`/`orientation` so the
//! constraint stays geometrically equivalent.
//!
//! This crate's [`PlanningRequest::goal_constraints`]/`path_constraints`
//! carry [`cspace_constraints::KinematicConstraintSet`], not a raw message —
//! and [`cspace_constraints::PositionConstraint::new`]/
//! [`cspace_constraints::OrientationConstraint::new`] both require
//! `link_name` to already name a real robot link
//! (`RobotModel::link_model(link_name)`, `cspace-constraints/src/position.rs`/
//! `orientation.rs`), erroring at construction otherwise. `RobotModel::link_model`
//! has no notion of an attached body or subframe (those live on
//! [`cspace_scene::PlanningScene`]/`AttachedBody`, a type `RobotModel` does
//! not reference) — so a subframe-named `link_name` can never exist in an
//! already-constructed [`cspace_constraints::PositionConstraint`]/
//! [`cspace_constraints::OrientationConstraint`] in this workspace at all.
//! By the time a constraint reaches this adapter, upstream's resolution
//! problem has already been forced to not exist, at construction time, by a
//! stricter constructor than upstream's raw-message path ever enforced.
//! Verified, not assumed: `rg -n 'pub fn new' crates/cspace-constraints/src/position.rs
//! crates/cspace-constraints/src/orientation.rs` shows both taking
//! `link_name: &str` and both calling `model.link_model(link_name)?` before
//! anything else.
//!
//! [`ResolveConstraintFrames::adapt`] therefore always returns `Ok(())`,
//! exactly matching upstream's unconditional `SUCCESS`, with nothing left to
//! rewrite.

use cspace_collision::ParryCollisionEnv;
use cspace_scene::PlanningScene;

use crate::PlanningRequestAdapter;
use crate::error::RequestAdapterError;
use crate::request::PlanningRequest;

/// See the module doc for why this is a structural no-op in this port.
#[derive(Debug, Default)]
pub struct ResolveConstraintFrames;

impl PlanningRequestAdapter for ResolveConstraintFrames {
    fn description(&self) -> &'static str {
        "ResolveConstraintFrames"
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
    use cspace_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
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

    #[test]
    fn leaves_an_already_link_scoped_constraint_set_unchanged() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut goal_set = KinematicConstraintSet::new();
        goal_set.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint1", 0.0, 0.01, 0.01, 1.0).unwrap(),
        ));
        let mut request = PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal_constraints: vec![goal_set.clone()],
            path_constraints: None,
            workspace_bounds: Default::default(),
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
            ..Default::default()
        };

        let adapter = ResolveConstraintFrames;
        assert_eq!(adapter.description(), "ResolveConstraintFrames");
        assert!(adapter.adapt(&mut scene, &env, &mut request).is_ok());
        assert_eq!(request.goal_constraints, vec![goal_set]);
    }
}
