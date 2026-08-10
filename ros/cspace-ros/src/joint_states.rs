// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The `joint_states` publisher a client's `getCurrentState()` waits on.
//!
//! # Which client call this serves, and which it does not
//!
//! Not `plan()`. `PORTING-PLAN.md` §273.5's reading holds where it matters:
//! `plan()` sends `considered_start_state_`, which
//! `setStartStateToCurrentState` leaves as an empty diff, so a plan is
//! answered whether or not anything ever published a joint state.
//!
//! What blocks is `getCurrentState()`
//! (`move_group_interface.cpp:635-655`). It starts the monitor if it is not
//! running (`:644-645`) and then calls
//! `waitForCurrentState(node_->now(), wait_seconds)` (`:647`) -- and *that*
//! call is why a latched message is not enough. `waitForCurrentState` loops
//! while `current_state_time_ < t` (`current_state_monitor.cpp:240`) where
//! `t` is the client's own `now()` at the moment of the call, and
//! `current_state_time_` is taken from the received message's
//! `header.stamp` (`:341`). A single retained sample carries the stamp it
//! was published with, which is by construction older than any later
//! `now()`. The publisher therefore has to keep publishing, with a fresh
//! stamp each time; hence the timer in `src/bin/move_group.rs` rather than
//! `moveit_ros::robot_description`'s single latched send.
//!
//! # What goes in the message
//!
//! `jointStateCallback` (`current_state_monitor.cpp:322-334`) drops the
//! whole message when `name.size() != position.size()`, so those two arrays
//! agreeing is not a detail -- it is the difference between the client
//! seeing this node's state and seeing nothing, with one throttled log line
//! upstream and none here. [`JointSampler::sample`] builds both from a
//! single `unzip`, so there is no arrangement of this module's inputs that
//! produces two lengths.
//!
//! Which joints: the callback skips any name whose joint has
//! `getVariableCount() != 1` (`:352-353`, "they should not even be in the
//! message"), and `haveCompleteStateHelper` (`:200-230`) requires *every*
//! `getActiveJointModels()` entry to have been stamped before
//! `haveCompleteState()` is true. [`JointSampler::new`] resolves exactly
//! that set once, at startup: the model's active joints, minus any that is
//! not single-DOF.
//!
//! That subtraction is a real limit and not a rounding: a model with a
//! multi-DOF active joint -- an SRDF `virtual_joint` of type `planar` or
//! `floating` -- can never reach `haveCompleteState()` from this topic
//! alone, because upstream fills those from TF instead
//! (`transformCallback`, reached from `startStateMonitor`'s
//! `createDynamicTfSubscription` at `:164-169`). This node publishes no TF,
//! so against such a model a client's `getCurrentState()` fails on its
//! timeout. `ros/fixtures/one_joint.srdf` declares no virtual joint, so
//! every active joint in the fixture is single-DOF and the set is not
//! empty; [`JointSampler::new`] rejects an empty set rather than publishing
//! a message that can never complete a state.
//!
//! # Deviation: upstream's `move_group` does not publish this
//!
//! It subscribes. `joint_states` is the robot driver's topic -- upstream's
//! `PlanningSceneMonitor` and every `MoveGroupInterface` read it, and
//! `move_group` never writes it. This node stands in for that driver,
//! publishing the monitored scene's own current state, so the client is told
//! the state this node plans from rather than the state of a robot that does
//! not exist. A real deployment would have both, and then this publisher
//! would be a second writer on a topic that must have exactly one.
//!
//! The values are the monitored scene's `current_state()`, which this node
//! only ever changes by applying a `planning_scene` message -- so what the
//! client reads back is what it (or another publisher) put there, not
//! motion. Nothing here integrates or simulates.

use cspace_core::model::RobotModel;
use cspace_core::state::RobotState;
use r2r::builtin_interfaces::msg::Time;
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::Header;

/// The active single-DOF joints of one model, resolved to variable indices
/// once so that sampling cannot fail.
///
/// Built at startup, where a model this node cannot describe on
/// `joint_states` is a reason to refuse to start; [`JointSampler::sample`]
/// is then total, which is what lets the publishing loop have no error arm
/// of its own.
pub struct JointSampler {
    /// `(joint name, index into `RobotState::positions`)`, one entry per
    /// active single-DOF joint. One vector rather than two, so the message's
    /// two arrays come from one `unzip` and cannot differ in length.
    joints: Vec<(String, usize)>,
}

/// Why a model cannot be described on `joint_states`.
#[derive(Debug)]
pub enum SamplerError {
    /// The model has no active single-DOF joint, so every message would be
    /// empty and `haveCompleteState()` could never become true.
    NoSingleDofJoints,
    /// A joint's own variable is not a variable of the model that owns it.
    /// Unreachable through [`RobotModel`]'s own constructors; named rather
    /// than unwrapped so that a future model builder that breaks the
    /// invariant is reported instead of panicking in a publishing loop.
    UnknownVariable(String),
}

impl std::fmt::Display for SamplerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoSingleDofJoints => write!(
                f,
                "the model has no active single-DOF joint, so a joint_states message built \
                 from it could never complete a client's current state"
            ),
            Self::UnknownVariable(name) => {
                write!(
                    f,
                    "joint variable {name:?} is not a variable of its own model"
                )
            }
        }
    }
}

impl JointSampler {
    /// Resolves `model`'s active single-DOF joints to variable indices.
    ///
    /// # Errors
    ///
    /// [`SamplerError::NoSingleDofJoints`] if the model has none;
    /// [`SamplerError::UnknownVariable`] if a joint's variable is not in its
    /// own model.
    pub fn new(model: &RobotModel) -> Result<Self, SamplerError> {
        let mut joints = Vec::new();
        for &index in model.active_joint_indices() {
            let joint = model.joint_model_at(index);
            // Upstream's own filter, at the far end of the wire
            // (`current_state_monitor.cpp:352-353`). Applying it here rather
            // than sending a name the client will discard keeps the two
            // arrays describing the same set of joints on both sides.
            if joint.variable_count() != 1 {
                continue;
            }
            // A single-variable joint's one variable is named after the
            // joint itself, so this is the name the client looks up with
            // `getJointModel(name[i])`.
            let variable = &joint.variable_names()[0];
            let position = model
                .variable_index(variable)
                .map_err(|_| SamplerError::UnknownVariable(variable.clone()))?;
            joints.push((joint.name().to_string(), position));
        }
        if joints.is_empty() {
            return Err(SamplerError::NoSingleDofJoints);
        }
        Ok(Self { joints })
    }

    /// The message for `state` at `stamp`.
    ///
    /// Total. `name` and `position` are the two halves of one `unzip` over
    /// this sampler's own joint list, so they have the same length by
    /// construction rather than by check -- see this module's doc for what
    /// upstream does to a message where they differ.
    ///
    /// `velocity` and `effort` are empty: upstream reads them only under
    /// `copy_dynamics_` (`current_state_monitor.cpp:385-401`), which nothing
    /// in the client's path sets, and an empty array is how a publisher says
    /// it has none.
    pub fn sample(&self, state: &RobotState<'_>, stamp: Time) -> JointState {
        let positions = state.positions();
        let (name, position): (Vec<String>, Vec<f64>) = self
            .joints
            .iter()
            .map(|(name, index)| (name.clone(), positions[*index]))
            .unzip();
        JointState {
            header: Header {
                stamp,
                // Upstream reads no frame from this message -- `jointStateCallback`
                // touches `header.stamp` and nothing else of the header
                // (`current_state_monitor.cpp:341`, `:355`). An invented frame
                // would be a claim about a TF tree this node does not publish.
                frame_id: String::new(),
            },
            name,
            position,
            velocity: Vec::new(),
            effort: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cspace_core::model::MeshSearchPaths;
    use cspace_core::srdf::SrdfModel;

    const ONE_JOINT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base_link"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>"#;

    const ONE_JOINT_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <group name="arm">
    <chain base_link="base_link" tip_link="tip"/>
  </group>
</robot>"#;

    /// A model with a `planar` virtual joint beside the revolute one: the
    /// active-joint set then holds a 3-variable joint this topic cannot
    /// carry, which is the case the module's doc names as a real limit.
    const PLANAR_VIRTUAL_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <virtual_joint name="world_joint" type="planar" parent_frame="odom" child_link="base_link"/>
  <group name="arm">
    <chain base_link="base_link" tip_link="tip"/>
  </group>
</robot>"#;

    /// A URDF whose only joint is fixed, so the model has no active joint at
    /// all.
    const FIXED_ONLY_URDF: &str = r#"<?xml version="1.0"?>
<robot name="fixed_only">
  <link name="base_link"/>
  <link name="tip"/>
  <joint name="j1" type="fixed">
    <parent link="base_link"/>
    <child link="tip"/>
  </joint>
</robot>"#;

    fn model(urdf_xml: &str, srdf_xml: &str) -> RobotModel {
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("fixture URDF parses");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("fixture SRDF parses");
        RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model builds")
    }

    fn stamp(sec: i32) -> Time {
        Time { sec, nanosec: 0 }
    }

    /// The property upstream drops the whole message over
    /// (`current_state_monitor.cpp:324-334`). Asserted on a model whose
    /// active set is *not* all single-DOF, because that is the only way the
    /// two arrays could have come from different filters.
    #[test]
    fn the_two_arrays_have_the_same_length_on_a_model_with_a_multi_dof_joint() {
        let model = model(ONE_JOINT_URDF, PLANAR_VIRTUAL_SRDF);
        let sampler = JointSampler::new(&model).expect("the revolute joint is single-DOF");
        let state = RobotState::new(&model);
        let message = sampler.sample(&state, stamp(7));
        assert_eq!(message.name.len(), message.position.len());
        assert_eq!(message.name.len(), 1, "got {:?}", message.name);
    }

    /// The multi-DOF joint is excluded by name, not merely by count: a
    /// filter that dropped the wrong joint would still leave one entry.
    #[test]
    fn a_multi_dof_virtual_joint_is_not_named_in_the_message() {
        let model = model(ONE_JOINT_URDF, PLANAR_VIRTUAL_SRDF);
        let sampler = JointSampler::new(&model).expect("the revolute joint is single-DOF");
        let state = RobotState::new(&model);
        let message = sampler.sample(&state, stamp(7));
        assert_eq!(message.name, vec!["j1".to_string()]);
        assert!(
            !message.name.iter().any(|n| n.starts_with("world_joint")),
            "a 3-variable joint reached the message: {:?}",
            message.name
        );
    }

    /// The name upstream looks up with `getJointModel(name[i])` is the
    /// *joint* name. Spelled out rather than derived from the model, so a
    /// change that started sending variable names (`world_joint/x`) fails
    /// here.
    #[test]
    fn the_names_are_joint_names_and_the_positions_are_that_joints_value() {
        let model = model(ONE_JOINT_URDF, ONE_JOINT_SRDF);
        let sampler = JointSampler::new(&model).expect("j1 is single-DOF");
        let mut state = RobotState::new(&model);
        state
            .set_variable_position("j1", 0.25)
            .expect("j1 is a variable of this model");
        let message = sampler.sample(&state, stamp(1));
        assert_eq!(message.name, vec!["j1".to_string()]);
        assert_eq!(message.position, vec![0.25]);
    }

    /// The stamp is the caller's, unchanged: it is what
    /// `waitForCurrentState` compares against, so a publisher that
    /// substituted its own would be answering a different question.
    #[test]
    fn the_stamp_is_the_one_the_caller_passed() {
        let model = model(ONE_JOINT_URDF, ONE_JOINT_SRDF);
        let sampler = JointSampler::new(&model).expect("j1 is single-DOF");
        let state = RobotState::new(&model);
        let message = sampler.sample(
            &state,
            Time {
                sec: 12,
                nanosec: 34,
            },
        );
        assert_eq!(message.header.stamp.sec, 12);
        assert_eq!(message.header.stamp.nanosec, 34);
    }

    /// A model this topic cannot describe is refused at construction rather
    /// than served as an empty message every tick.
    #[test]
    fn a_model_with_no_active_single_dof_joint_is_refused() {
        let model = model(FIXED_ONLY_URDF, ONE_JOINT_SRDF);
        let error = JointSampler::new(&model)
            .err()
            .expect("a fixed-only model has no active single-DOF joint");
        assert!(
            matches!(error, SamplerError::NoSingleDofJoints),
            "got {error:?}"
        );
    }

    /// `velocity` and `effort` stay empty. Upstream reads them only under
    /// `copy_dynamics_`, and a zero-filled array would be a claim that the
    /// robot is stationary rather than that nothing measured it.
    #[test]
    fn no_velocity_or_effort_is_claimed() {
        let model = model(ONE_JOINT_URDF, ONE_JOINT_SRDF);
        let sampler = JointSampler::new(&model).expect("j1 is single-DOF");
        let state = RobotState::new(&model);
        let message = sampler.sample(&state, stamp(1));
        assert!(message.velocity.is_empty());
        assert!(message.effort.is_empty());
    }
}
