// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/RobotState` <-> [`moveit_state::RobotState`] (round 2,
//! PORTING-PLAN.md Phase 9). See `doc/message-mapping.md` §9 for the full
//! survey this module codes against.
//!
//! There is no core type isomorphic to the wire `RobotState` -- only its
//! `joint_state: sensor_msgs/JointState` field is representable at all.
//! `multi_dof_joint_state`, `attached_collision_objects` and `is_diff` are
//! genuine structural gaps (§9), so msg->core **rejects** (does not silently
//! drop) any message that actually uses them, per D6: a `TryFrom` that
//! quietly ignored data the caller supplied would be exactly the "failure
//! absorbed into a silent default" D6 exists to prevent. A caller that needs
//! those fields has to compose with `moveit-scene`/`moveit-state` directly,
//! one level up from what a bare `&RobotModel` conversion can see.
//!
//! # Expiry conditions (PORTING-PLAN.md §153.1: name what clears each gap)
//!
//! - `multi_dof_joint_state`: expires if `moveit_state::RobotState` (or a
//!   sibling type) gains a way to represent multi-DOF joint values --
//!   today its variable space comes entirely from `moveit_model::RobotModel`'s
//!   single-DOF variables.
//! - `attached_collision_objects`/`is_diff`: **not** a missing core type --
//!   `PlanningScene` already carries attached bodies and parent-diffing.
//!   These expire if this crate adds a conversion entry point that takes
//!   `&mut PlanningScene` alongside the message (the composed conversion
//!   named above, not attempted this round), not if `moveit-state` changes.
//!
//! Neither condition above can be turned into a tripwire (round 13:
//! D14/§199 proved the pattern works -- checked here whether it applies).
//! A tripwire needs an *existing* call path whose current answer would
//! change; both of these name the *arrival* of a capability that has no
//! call path yet to assert anything about:
//! `moveit_model`/`moveit_state` have no multi-DOF-joint symbol at all
//! today (checked: no `multi_dof`/`MultiDof` hit anywhere in
//! `crates/moveit-model/src`), and this crate itself has no
//! `&mut PlanningScene`-aware conversion function to call into yet. A
//! runtime assertion cannot test for the absence of an API that does not
//! exist -- there is nothing to invoke and watch fail. Contrast
//! `trajectory.rs`'s nonzero-start-time gap, which *is* now tripwired,
//! because `add_suffix_way_point` already exists and already enforces
//! the invariant today.

use moveit_error::Error;
use moveit_model::RobotModel;
use moveit_state::RobotState as CoreRobotState;
use r2r::moveit_msgs::msg as moveit_msgs;
use r2r::sensor_msgs::msg as sensor_msgs;

/// Wraps `moveit_msgs::msg::RobotState` together with the `&RobotModel`
/// needed to resolve `joint_state.name[]` into variable indices (see this
/// crate's `lib.rs` doc comment on the orphan-rule wrapper convention -- this
/// one also carries context, not just the message, because the message alone
/// is not enough to build a [`CoreRobotState`], same shape as
/// `doc/message-mapping.md`'s per-row notes on frame-lookup conversions).
pub struct RobotStateMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: moveit_msgs::RobotState,
}

/// Wraps `moveit_msgs::msg::RobotState` as a plain local newtype, for the
/// core->msg direction (no extra context needed to serialize).
pub struct RobotStateMsgOut(pub moveit_msgs::RobotState);

fn set_parallel_array(
    state: &mut CoreRobotState,
    names: &[String],
    values: &[f64],
    field: &'static str,
    set_by_name: impl Fn(&mut CoreRobotState, &str, f64) -> moveit_error::Result<()>,
) -> moveit_error::Result<()> {
    if !values.is_empty() && values.len() != names.len() {
        return Err(Error::construct(format!(
            "JointState.{field} has length {} but name has length {} \
             (the wire's own convention: \"All arrays in this message \
             should have the same size, or be empty\" -- this message \
             violates it)",
            values.len(),
            names.len()
        )));
    }
    for (name, &value) in names.iter().zip(values.iter()) {
        set_by_name(state, name, value)?;
    }
    Ok(())
}

impl<'m> TryFrom<RobotStateMsg<'m>> for CoreRobotState<'m> {
    type Error = Error;

    fn try_from(wrapped: RobotStateMsg<'m>) -> Result<Self, Self::Error> {
        let RobotStateMsg { model, msg } = wrapped;

        if msg.is_diff {
            return Err(Error::other(
                "RobotState.is_diff=true needs a parent PlanningScene to \
                 diff against, which a bare &RobotModel conversion has no \
                 access to (see doc/message-mapping.md §9/§11)",
            ));
        }
        if !msg.attached_collision_objects.is_empty() {
            return Err(Error::other(
                "RobotState.attached_collision_objects is not \
                 representable here: moveit-rs keeps attached bodies on \
                 PlanningScene, not RobotState (attached_body.rs, see \
                 doc/message-mapping.md §9)",
            ));
        }
        let mdjs = &msg.multi_dof_joint_state;
        if !mdjs.joint_names.is_empty()
            || !mdjs.transforms.is_empty()
            || !mdjs.twist.is_empty()
            || !mdjs.wrench.is_empty()
        {
            return Err(Error::other(
                "RobotState.multi_dof_joint_state has no core \
                 representation this round (see doc/message-mapping.md §9)",
            ));
        }

        let js = msg.joint_state;
        let mut state = CoreRobotState::new(model);
        set_parallel_array(&mut state, &js.name, &js.position, "position", |s, n, v| {
            s.set_variable_position(n, v)
        })?;
        set_parallel_array(&mut state, &js.name, &js.velocity, "velocity", |s, n, v| {
            s.set_variable_velocity(n, v)
        })?;
        set_parallel_array(&mut state, &js.name, &js.effort, "effort", |s, n, v| {
            s.set_variable_effort(n, v)
        })?;
        Ok(state)
    }
}

impl<'m> TryFrom<CoreRobotState<'m>> for RobotStateMsgOut {
    type Error = Error;

    /// Total. `sensor_msgs/JointState` has no `acceleration` field at all
    /// (confirmed against the generated bindings) -- core's
    /// `accelerations()` has no wire home on this message and is dropped,
    /// a core-only-field gap not previously named in
    /// `doc/message-mapping.md` §9 (that section covered wire-only gaps;
    /// this is the reverse). `multi_dof_joint_state`/
    /// `attached_collision_objects` are emitted empty and `is_diff` is
    /// `false`: a bare `RobotState` carries no parent-scene or multi-DOF
    /// information to source them from, so there is nothing to lose here
    /// that a fuller conversion (composed with `&PlanningScene`, not
    /// attempted this round) wouldn't recover.
    fn try_from(state: CoreRobotState<'m>) -> Result<Self, Self::Error> {
        let names = state.model().variable_names().to_vec();
        Ok(RobotStateMsgOut(moveit_msgs::RobotState {
            joint_state: sensor_msgs::JointState {
                header: Default::default(),
                name: names,
                position: state.positions().to_vec(),
                velocity: if state.has_velocities() {
                    state.velocities().to_vec()
                } else {
                    Vec::new()
                },
                effort: if state.has_effort() {
                    state.effort().to_vec()
                } else {
                    Vec::new()
                },
            },
            multi_dof_joint_state: Default::default(),
            attached_collision_objects: Vec::new(),
            is_diff: false,
        }))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use moveit_model::MeshSearchPaths;
    use moveit_srdf::SrdfModel;

    pub(crate) fn one_joint_model() -> RobotModel {
        let urdf_xml = r#"<?xml version="1.0"?>
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
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="one_joint">
</robot>
"#;
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("inline URDF must parse");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("inline SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("valid single-joint urdf")
    }

    #[test]
    fn joint_state_positions_convert_by_name() {
        let model = one_joint_model();
        let msg = moveit_msgs::RobotState {
            joint_state: sensor_msgs::JointState {
                header: Default::default(),
                name: vec!["j1".to_string()],
                position: vec![0.5],
                velocity: vec![],
                effort: vec![],
            },
            multi_dof_joint_state: Default::default(),
            attached_collision_objects: vec![],
            is_diff: false,
        };
        let state = CoreRobotState::try_from(RobotStateMsg { model: &model, msg }).unwrap();
        assert_eq!(state.variable_position("j1").unwrap(), 0.5);
        assert!(!state.has_velocities());
    }

    #[test]
    fn mismatched_position_length_is_rejected() {
        let model = one_joint_model();
        let msg = moveit_msgs::RobotState {
            joint_state: sensor_msgs::JointState {
                header: Default::default(),
                name: vec!["j1".to_string(), "j2".to_string()],
                position: vec![0.5],
                velocity: vec![],
                effort: vec![],
            },
            multi_dof_joint_state: Default::default(),
            attached_collision_objects: vec![],
            is_diff: false,
        };
        let err = CoreRobotState::try_from(RobotStateMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn unknown_joint_name_is_rejected() {
        let model = one_joint_model();
        let msg = moveit_msgs::RobotState {
            joint_state: sensor_msgs::JointState {
                header: Default::default(),
                name: vec!["no_such_joint".to_string()],
                position: vec![0.0],
                velocity: vec![],
                effort: vec![],
            },
            multi_dof_joint_state: Default::default(),
            attached_collision_objects: vec![],
            is_diff: false,
        };
        let err = CoreRobotState::try_from(RobotStateMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::UnknownName { .. }), "got: {err:?}");
    }

    #[test]
    fn is_diff_is_rejected_not_silently_dropped() {
        let model = one_joint_model();
        let msg = moveit_msgs::RobotState {
            joint_state: Default::default(),
            multi_dof_joint_state: Default::default(),
            attached_collision_objects: vec![],
            is_diff: true,
        };
        let err = CoreRobotState::try_from(RobotStateMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
    }

    #[test]
    fn round_trip_through_msg() {
        let model = one_joint_model();
        let mut state = CoreRobotState::new(&model);
        state.set_variable_position("j1", 0.25).unwrap();
        state.set_variable_velocity("j1", 1.5).unwrap();
        let msg = RobotStateMsgOut::try_from(state).unwrap().0;
        let back = CoreRobotState::try_from(RobotStateMsg { model: &model, msg }).unwrap();
        assert_eq!(back.variable_position("j1").unwrap(), 0.25);
        assert_eq!(back.variable_velocity("j1").unwrap(), 1.5);
    }
}
