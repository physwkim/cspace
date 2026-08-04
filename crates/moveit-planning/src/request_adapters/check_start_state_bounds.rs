// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_request_adapter_plugins/src/check_start_state_bounds.cpp

//! `default_planning_request_adapters::CheckStartStateBounds`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (cpp:46-208)
//!
//! `rclcpp` — 8 occurrences, all logging or ROS-parameter plumbing,
//! discarded (no logging dependency in this crate; parameters are a plain
//! `bool` argument here instead of a `ParamListener`, see "Deviation" below):
//! `rclcpp/logger.hpp`/`logging.hpp`/`node.hpp`/`parameter_value.hpp`
//! includes (cpp:50-53), the `rclcpp::Node::SharedPtr` parameter of
//! `initialize` (cpp:71, not ported — see "Deviation"), one `RCLCPP_DEBUG`
//! (cpp:84), the `rclcpp::Logger logger_` field (cpp:203) and constructor
//! initializer (cpp:65), one `RCLCPP_WARN` (cpp:195).
//!
//! `moveit_msgs` — 1 occurrence, computation, ported:
//! `moveit_msgs::msg::MoveItErrorCodes::{SUCCESS, START_STATE_INVALID}`
//! (cpp:184, cpp:188) — [`RequestAdapterError::StartStateInvalid`] and
//! `Ok(())` respectively.
//!
//! # Deviation: `fix_start_state` is a plain field, not a ROS parameter
//!
//! Upstream reads `params.fix_start_state` from a `generate_parameter_library`
//! `ParamListener` built in `initialize(rclcpp::Node::SharedPtr, ...)` — a
//! ROS type in the signature, D1. [`CheckStartStateBounds::new`] takes the
//! same boolean directly instead; a caller that would have set it via a ROS
//! parameter YAML file passes it as a constructor argument here.
//!
//! # Ported computation
//!
//! Every joint in `request.group_name` (falling back to every joint in the
//! model if the group is unknown, matching cpp:92-94's own
//! `hasJointModelGroup(...) ? ... : getJointModels()` fallback) is, in
//! order:
//!
//! 1. If revolute *and* continuous: wrapped into `[-pi, pi]` via
//!    [`moveit_model::JointModel::enforce_position_bounds`] — safe to call
//!    unconditionally only for a continuous joint, since that variant of
//!    `enforce_position_bounds` never clamps (see `moveit-model`'s
//!    `revolute.rs`, `enforce_position_bounds_wraps_when_continuous`); this
//!    adapter must never silently clamp a genuinely out-of-bounds *bounded*
//!    joint the way the whole-state `enforce_bounds()` would, so this port
//!    calls the per-joint dispatcher only inside this `is_continuous()`
//!    guard, exactly mirroring cpp:112's own guard.
//! 2. If planar: `values[2]` (yaw) renormalized via
//!    [`moveit_model::PlanarJoint::normalize_rotation`] (cpp:129).
//! 3. If floating: the quaternion renormalized via
//!    [`moveit_model::FloatingJoint::normalize_rotation`] (cpp:141).
//! 4. Regardless of type: checked against its own bounds via
//!    [`moveit_model::JointModel::satisfies_position_bounds`] with
//!    `margin = 0.0`. Upstream's own default here is not
//!    `satisfiesPositionBounds` in isolation but
//!    `RobotState::satisfiesBounds(jmodel, margin)`
//!    (`moveit/robot_state/robot_state.hpp:1419`):
//!    `satisfiesPositionBounds(joint, margin) && (!has_velocity_ ||
//!    satisfiesVelocityBounds(joint, margin))`. A prior version of this
//!    port called only the position half, silently accepting a start state
//!    whose velocities were set and out of bounds. Fixed: step 4 now also
//!    checks [`moveit_model::JointModel::satisfies_velocity_bounds`]
//!    against [`moveit_state::RobotState::joint_velocity`] whenever
//!    [`moveit_state::RobotState::has_velocities`] is true, mirroring the
//!    `has_velocity_` conditional exactly — [`moveit_state::RobotState`]
//!    already carries a `has_velocity`/`velocity` pair for this (ported
//!    separately, `crates/moveit-state/src/state.rs`), so there is no
//!    "does this port even have the concept" question to answer: the
//!    input this adapter was missing was already there to read.
//!
//! `should_fix_state` (a step-1/2/3 change was made) and `is_out_of_bounds`
//! (a step-4 check failed) are tracked exactly as cpp:99-100/119/132/144/157
//! do. The final decision (cpp:186-197) is unchanged: real bound violations
//! (`is_out_of_bounds`) always reject, regardless of `fix_start_state` — this
//! adapter never auto-corrects a joint actually outside its limits, only a
//! continuous-wrap/quaternion-renormalization representation change, and
//! only when `fix_start_state` allows it.

use moveit_collision::ParryCollisionEnv;
use moveit_model::joint::{FloatingJoint, JointType, PlanarJoint};
use moveit_scene::PlanningScene;

use crate::PlanningRequestAdapter;
use crate::error::RequestAdapterError;
use crate::request::PlanningRequest;

/// See the module doc. `fix_start_state` replaces upstream's
/// `params.fix_start_state` ROS parameter.
#[derive(Debug, Clone, Copy)]
pub struct CheckStartStateBounds {
    fix_start_state: bool,
}

impl CheckStartStateBounds {
    /// `fix_start_state`: whether a continuous-joint wrap or quaternion
    /// renormalization may be silently applied to the scene's current state
    /// (cpp:191-197), rather than rejected the same as a real bound
    /// violation (cpp:186-190).
    pub fn new(fix_start_state: bool) -> Self {
        Self { fix_start_state }
    }
}

impl PlanningRequestAdapter for CheckStartStateBounds {
    fn description(&self) -> &'static str {
        "CheckStartStateBounds"
    }

    fn adapt<'m>(
        &self,
        scene: &mut PlanningScene<'m>,
        _env: &ParryCollisionEnv,
        request: &mut PlanningRequest,
    ) -> Result<(), RequestAdapterError> {
        let model = scene.robot_model();
        let joint_indices: Vec<usize> = match model.joint_model_group(&request.group_name) {
            Ok(group) => group.active_joint_indices().to_vec(),
            Err(_) => (0..model.joint_models().count()).collect(),
        };

        let mut should_fix_state = false;
        let mut is_out_of_bounds = false;
        let state = scene.current_state_mut();

        for joint_index in joint_indices {
            let joint = model.joint_model_at(joint_index);
            let variable_count = joint.variable_count();
            if variable_count == 0 {
                continue;
            }
            let name = joint.name().to_string();
            let mut values = state.joint_position(&name).unwrap().to_vec();

            match joint.joint_type() {
                JointType::Revolute if joint.as_revolute().unwrap().is_continuous() => {
                    if joint.enforce_position_bounds(&mut values) {
                        should_fix_state = true;
                    }
                }
                JointType::Planar => {
                    let planar: &mut [f64; 3] = (&mut values[..3]).try_into().unwrap();
                    if PlanarJoint::normalize_rotation(planar) {
                        should_fix_state = true;
                    }
                }
                JointType::Floating => {
                    let floating: &mut [f64; 7] = (&mut values[..7]).try_into().unwrap();
                    if FloatingJoint::normalize_rotation(floating) {
                        should_fix_state = true;
                    }
                }
                _ => {}
            }

            if !joint.satisfies_position_bounds(&values, 0.0) {
                is_out_of_bounds = true;
            }
            if state.has_velocities()
                && !joint.satisfies_velocity_bounds(state.joint_velocity(&name).unwrap(), 0.0)
            {
                is_out_of_bounds = true;
            }

            state.set_joint_positions(&name, &values).unwrap();
        }

        if is_out_of_bounds || (!self.fix_start_state && should_fix_state) {
            return Err(RequestAdapterError::StartStateInvalid {
                adapter: self.description(),
            });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use std::f64::consts::PI;
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

    /// Panda (`fixtures/panda.urdf`) has no continuous joint at all — every
    /// revolute joint there is bounded. `fixtures/pr2.urdf` does
    /// (`r_forearm_roll_joint`/`r_wrist_roll_joint`, among others), so the
    /// continuous-wrap test below needs this fixture instead of [`panda`].
    fn pr2() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/pr2.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/pr2.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// `group_name` is `panda`'s own `panda_arm` group. [`pr2`] has no group
    /// by that name, so a test that reuses this request against a [`pr2`]
    /// model deliberately falls back to
    /// [`CheckStartStateBounds::adapt`]'s documented all-model-joints path
    /// instead of a narrower active-joint set.
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
    fn a_default_pose_within_bounds_is_accepted() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let adapter = CheckStartStateBounds::new(false);
        assert_eq!(adapter.description(), "CheckStartStateBounds");
        assert!(adapter.adapt(&mut scene, &env, &mut request()).is_ok());
    }

    /// Boundary: [`moveit_state::RobotState::has_velocities`] false (no
    /// velocity ever set) must not spuriously fail the velocity half of
    /// step 4 -- covered implicitly by every other test here too, since
    /// none of them call `set_variable_velocity`, but stated as its own
    /// case since it is the boundary the velocity check's `has_velocities()`
    /// guard exists for.
    #[test]
    fn no_velocity_present_is_accepted() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        assert!(!scene.current_state().has_velocities());
        assert!(
            CheckStartStateBounds::new(false)
                .adapt(&mut scene, &env, &mut request())
                .is_ok()
        );
    }

    /// Boundary: a velocity within `[min_velocity, max_velocity]` must not
    /// reject a state whose position is otherwise fine.
    #[test]
    fn an_in_bounds_velocity_is_accepted() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let bounds = model.joint_model("panda_joint1").unwrap().variable_bounds()[0];
        assert!(
            bounds.velocity_bounded,
            "fixture must have a velocity limit for this to test anything"
        );
        scene
            .current_state_mut()
            .set_variable_velocity("panda_joint1", bounds.max_velocity)
            .unwrap();
        assert!(scene.current_state().has_velocities());
        assert!(
            CheckStartStateBounds::new(false)
                .adapt(&mut scene, &env, &mut request())
                .is_ok()
        );
    }

    /// Boundary: a velocity outside `[min_velocity, max_velocity]` must
    /// reject, matching `robot_state.hpp:1419`'s
    /// `satisfiesVelocityBounds` half of `satisfiesBounds` -- the gap this
    /// port had before this round's fix (position-only, velocity never
    /// checked at all).
    #[test]
    fn an_out_of_bounds_velocity_is_rejected() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let bounds = model.joint_model("panda_joint1").unwrap().variable_bounds()[0];
        assert!(
            bounds.velocity_bounded,
            "fixture must have a velocity limit for this to test anything"
        );
        scene
            .current_state_mut()
            .set_variable_velocity("panda_joint1", bounds.max_velocity + 1.0)
            .unwrap();
        assert_eq!(
            CheckStartStateBounds::new(false).adapt(&mut scene, &env, &mut request()),
            Err(RequestAdapterError::StartStateInvalid {
                adapter: "CheckStartStateBounds"
            })
        );
    }

    #[test]
    fn a_joint_placed_outside_its_limits_is_rejected_regardless_of_fix_start_state() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let bounds = model.joint_model("panda_joint1").unwrap().variable_bounds()[0];
        scene
            .current_state_mut()
            .set_joint_positions("panda_joint1", &[bounds.max_position + 1.0])
            .unwrap();

        for fix_start_state in [false, true] {
            let mut scene = PlanningScene::new(&model, &srdf);
            scene
                .current_state_mut()
                .set_joint_positions("panda_joint1", &[bounds.max_position + 1.0])
                .unwrap();
            let adapter = CheckStartStateBounds::new(fix_start_state);
            assert_eq!(
                adapter.adapt(&mut scene, &env, &mut request()),
                Err(RequestAdapterError::StartStateInvalid {
                    adapter: "CheckStartStateBounds"
                })
            );
        }
    }

    #[test]
    fn a_continuous_joint_past_pi_is_wrapped_and_accepted_only_when_fix_start_state_is_set() {
        let (model, srdf) = pr2();
        let env = ParryCollisionEnv::default();
        let continuous_joint = model
            .joint_models()
            .find(|j| j.as_revolute().map(|r| r.is_continuous()).unwrap_or(false))
            .expect(
                "fixture must have at least one continuous joint for this test to mean anything",
            )
            .name()
            .to_string();

        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_joint_positions(&continuous_joint, &[PI + 0.1])
            .unwrap();
        let rejected = CheckStartStateBounds::new(false)
            .adapt(&mut scene, &env, &mut request())
            .unwrap_err();
        assert_eq!(
            rejected,
            RequestAdapterError::StartStateInvalid {
                adapter: "CheckStartStateBounds"
            }
        );

        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_joint_positions(&continuous_joint, &[PI + 0.1])
            .unwrap();
        CheckStartStateBounds::new(true)
            .adapt(&mut scene, &env, &mut request())
            .expect("fix_start_state = true must accept a wrap-only change");
        let wrapped = scene
            .current_state()
            .joint_position(&continuous_joint)
            .unwrap()[0];
        assert!((-PI..=PI).contains(&wrapped), "wrapped = {wrapped}");
    }
}
