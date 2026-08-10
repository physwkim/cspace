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
//!    [`cspace_core::model::joint::JointModel::enforce_position_bounds`] — safe to call
//!    unconditionally only for a continuous joint, since that variant of
//!    `enforce_position_bounds` never clamps (see `cspace-model`'s
//!    `revolute.rs`, `enforce_position_bounds_wraps_when_continuous`); this
//!    adapter must never silently clamp a genuinely out-of-bounds *bounded*
//!    joint the way the whole-state `enforce_bounds()` would, so this port
//!    calls the per-joint dispatcher only inside this `is_continuous()`
//!    guard, exactly mirroring cpp:112's own guard. Whether this step
//!    counts as a `should_fix_state` change is decided by comparing the
//!    position *before* and *after* the call (cpp:112-119), not by
//!    `enforce_position_bounds`'s return value: that value is
//!    unconditionally `true` for a continuous joint regardless of whether
//!    the position actually moved (see the same doc comment above), so a
//!    prior version of this port that treated it as "did this change
//!    anything" rejected every start state with a continuous joint under
//!    `fix_start_state = false`, wrap needed or not. Fixed.
//! 2. If planar: `values[2]` (yaw) renormalized via
//!    [`cspace_core::model::joint::PlanarJoint::normalize_rotation`] (cpp:129).
//! 3. If floating: the quaternion renormalized via
//!    [`cspace_core::model::joint::FloatingJoint::normalize_rotation`] (cpp:141).
//! 4. Regardless of type: checked against its own bounds via
//!    [`cspace_core::model::joint::JointModel::satisfies_position_bounds`] with
//!    `margin = 0.0`. Upstream's own default here is not
//!    `satisfiesPositionBounds` in isolation but
//!    `RobotState::satisfiesBounds(jmodel, margin)`
//!    (`moveit/robot_state/robot_state.hpp:1419`):
//!    `satisfiesPositionBounds(joint, margin) && (!has_velocity_ ||
//!    satisfiesVelocityBounds(joint, margin))`. A prior version of this
//!    port called only the position half, silently accepting a start state
//!    whose velocities were set and out of bounds. Fixed: step 4 now also
//!    checks [`cspace_core::model::joint::JointModel::satisfies_velocity_bounds`]
//!    against [`cspace_core::state::RobotState::joint_velocity`] whenever
//!    [`cspace_core::state::RobotState::has_velocities`] is true, mirroring the
//!    `has_velocity_` conditional exactly — [`cspace_core::state::RobotState`]
//!    already carries a `has_velocity`/`velocity` pair for this (ported
//!    separately, `crates/cspace-core/src/state/state.rs`), so there is no
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

use cspace_collision::ParryCollisionEnv;
use cspace_core::model::joint::{FloatingJoint, JointType, PlanarJoint};
use cspace_scene::PlanningScene;

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
        // `joint_indices()`, not `active_joint_indices()`: upstream's
        // `getJointModelGroup(...)->getJointModels()` includes mimic (and
        // fixed) joints, matching the model-wide fallback below, which
        // also iterates every joint unfiltered. Using the active-only set
        // here silently skipped the bounds check for any mimic joint in
        // the group.
        let joint_indices: Vec<usize> = match model.joint_model_group(&request.group_name) {
            Ok(group) => group.joint_indices().to_vec(),
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
                    // `enforce_position_bounds` always returns `true` for a
                    // continuous joint (`cspace-model`'s `revolute.rs`),
                    // whether or not it actually changed `values` -- unlike
                    // `PlanarJoint`/`FloatingJoint::normalize_rotation`
                    // below, whose return value genuinely means "changed
                    // something". Upstream (cpp:112-119) compares the
                    // before/after position itself, not a return value, so
                    // this does too.
                    let before = values[0];
                    joint.enforce_position_bounds(&mut values);
                    if (values[0] - before).abs() > f64::EPSILON {
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
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
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

    /// Single-`planar`-joint fixture, not a vendored robot -- purpose-built
    /// so `should_fix_state`'s planar-renormalization cause (step 2 of the
    /// module doc's three, cpp:129) has a model to run against at all;
    /// neither `panda` nor `pr2` has a planar joint. No group named
    /// `panda_arm` exists here either, so [`request`] against this model
    /// exercises the same all-model-joints fallback [`pr2`]'s doc comment
    /// describes.
    fn planar_robot() -> (RobotModel, SrdfModel) {
        let urdf_xml = r#"<robot name="test">
            <link name="base"/>
            <link name="tip"/>
            <joint name="planar_joint" type="planar">
                <parent link="base"/>
                <child link="tip"/>
                <axis xyz="0 0 1"/>
            </joint>
        </robot>"#;
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_str(
            r#"<robot name="test">
                <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            </robot>"#,
        )
        .unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// Single-`floating`-joint fixture, not a vendored robot -- mirrors
    /// [`planar_robot`] but for `should_fix_state`'s quaternion-
    /// renormalization cause (step 3 of the module doc's three, cpp:141).
    /// Neither `panda` nor `pr2` has a floating joint.
    fn floating_robot() -> (RobotModel, SrdfModel) {
        let urdf_xml = r#"<robot name="test">
            <link name="base"/>
            <link name="tip"/>
            <joint name="floating_joint" type="floating">
                <parent link="base"/>
                <child link="tip"/>
            </joint>
        </robot>"#;
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_str(
            r#"<robot name="test">
                <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            </robot>"#,
        )
        .unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// Purpose-built so `mimic_group` (below) has exactly one member and it
    /// is a mimic joint: `active_joint_indices` excludes mimic joints, so
    /// for this group it is empty, while `joint_indices` (every joint,
    /// mimic ones included) has the one entry. This adapter's found-group
    /// branch reads one of these two sets (see this module's doc comment
    /// and `adapt`'s implementation) -- reading the wrong one skips this
    /// group's bounds check entirely, not just one joint of several.
    /// `drive_joint` is not itself a group member: including it would let
    /// its own per-joint write-back inside `adapt`'s loop re-propagate
    /// `mimic_joint`'s mimicked value and overwrite whatever this test set
    /// on `mimic_joint` directly, before the loop ever reaches checking it
    /// -- neither `panda` nor `pr2`'s groups isolate a mimic joint this
    /// cleanly.
    fn mimic_robot() -> (RobotModel, SrdfModel) {
        let urdf_xml = r#"<robot name="test">
            <link name="base"/>
            <link name="mid"/>
            <link name="tip"/>
            <joint name="drive_joint" type="revolute">
                <parent link="base"/>
                <child link="mid"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
            <joint name="mimic_joint" type="revolute">
                <parent link="mid"/>
                <child link="tip"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
                <mimic joint="drive_joint" multiplier="1.0" offset="0.0"/>
            </joint>
        </robot>"#;
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_str(
            r#"<robot name="test">
                <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
                <group name="mimic_group">
                    <joint name="mimic_joint"/>
                </group>
            </robot>"#,
        )
        .unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
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

    /// Boundary: [`cspace_core::state::RobotState::has_velocities`] false (no
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

    /// The found-group branch (`request.group_name` resolves) must check
    /// every joint in the group, mimic joints included -- upstream's
    /// `getJointModelGroup(...)->getJointModels()` does not exclude them,
    /// matching the model-wide fallback branch, which iterates
    /// `model.joint_models()` (also unfiltered). A mimic joint set outside
    /// its own limits must still be rejected.
    #[test]
    fn a_mimic_joint_placed_outside_its_limits_is_rejected() {
        let (model, srdf) = mimic_robot();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let bounds = model.joint_model("mimic_joint").unwrap().variable_bounds()[0];
        scene
            .current_state_mut()
            .set_joint_positions("mimic_joint", &[bounds.max_position + 1.0])
            .unwrap();

        let mut req = PlanningRequest {
            group_name: "mimic_group".to_string(),
            ..request()
        };
        let adapter = CheckStartStateBounds::new(false);
        assert_eq!(
            adapter.adapt(&mut scene, &env, &mut req),
            Err(RequestAdapterError::StartStateInvalid {
                adapter: "CheckStartStateBounds"
            })
        );
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

    /// Boundary the existing wrap test above never exercises: a continuous
    /// joint already inside `[-PI, PI]` needs no wrap at all.
    /// `enforce_position_bounds` always returns `true` for a continuous
    /// joint, whether or not it actually changed anything (see
    /// `cspace-model`'s `revolute.rs`, `enforce_position_bounds`'s own doc
    /// comment) -- trusting that return value as "did this change the
    /// state" rejected every start state containing a continuous joint
    /// under `fix_start_state = false`, upstream's own default, regardless
    /// of whether the joint needed wrapping at all.
    #[test]
    fn a_continuous_joint_already_in_bounds_needs_no_fix() {
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
            .set_joint_positions(&continuous_joint, &[0.5])
            .unwrap();
        assert_eq!(
            CheckStartStateBounds::new(false).adapt(&mut scene, &env, &mut request()),
            Ok(()),
            "an already-in-bounds continuous joint must not be rejected under \
             fix_start_state = false"
        );
    }

    /// Isolating mutation (assertion-discrimination sweep, round 15):
    /// `should_fix_state` funnels three independent causes into one bare
    /// `Err(StartStateInvalid)` (module doc, steps 1-3) -- continuous-joint
    /// wrap, planar renormalization, floating quaternion renormalization.
    /// The wrap cause above had a test; these two did not, at all, so
    /// there was nothing for a mutation to even confirm.
    #[test]
    fn a_planar_joint_past_pi_is_wrapped_and_accepted_only_when_fix_start_state_is_set() {
        let (model, srdf) = planar_robot();
        let env = ParryCollisionEnv::default();

        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_joint_positions("planar_joint", &[0.0, 0.0, PI + 0.1])
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
            .set_joint_positions("planar_joint", &[0.0, 0.0, PI + 0.1])
            .unwrap();
        CheckStartStateBounds::new(true)
            .adapt(&mut scene, &env, &mut request())
            .expect("fix_start_state = true must accept a wrap-only change");
        let wrapped = scene
            .current_state()
            .joint_position("planar_joint")
            .unwrap()[2];
        assert!((-PI..=PI).contains(&wrapped), "wrapped = {wrapped}");
    }

    /// See `a_planar_joint_past_pi_is_wrapped_...`'s doc comment -- same
    /// funnel, the third of its three causes.
    #[test]
    fn a_floating_joint_with_a_non_unit_quaternion_is_normalized_and_accepted_only_when_fix_start_state_is_set()
     {
        let (model, srdf) = floating_robot();
        let env = ParryCollisionEnv::default();

        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_joint_positions("floating_joint", &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
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
            .set_joint_positions("floating_joint", &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])
            .unwrap();
        CheckStartStateBounds::new(true)
            .adapt(&mut scene, &env, &mut request())
            .expect("fix_start_state = true must accept a normalize-only change");
        let quaternion = &scene
            .current_state()
            .joint_position("floating_joint")
            .unwrap()[3..7];
        let norm_sqr: f64 = quaternion.iter().map(|v| v * v).sum();
        assert!((norm_sqr - 1.0).abs() <= 1e-9, "norm_sqr = {norm_sqr}");
    }
}
