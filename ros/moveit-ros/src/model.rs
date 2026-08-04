// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/JointLimits` <-> [`moveit_model::JointLimits`].
//! See `doc/message-mapping.md` §3.

use moveit_error::Error;
use moveit_model::joint::JointLimits as CoreJointLimits;
use r2r::moveit_msgs::msg as moveit_msgs;

/// Orphan-rule wrapper for the msg->core direction (`src/lib.rs`'s doc
/// comment: every `TryFrom` here targets a local newtype, never the bare
/// `r2r` type against a bare core type directly).
pub struct JointLimitsMsg(pub moveit_msgs::JointLimits);

/// Wrapper for the core->msg direction.
pub struct JointLimitsMsgOut(pub moveit_msgs::JointLimits);

impl TryFrom<JointLimitsMsg> for CoreJointLimits {
    type Error = Error;

    /// Total in both directions: every field is name- and type-identical on
    /// both sides (§3), confirmed against the actual `moveit_msgs/msg/
    /// JointLimits.msg` (10 fields -- `joint_name` plus a `has_*_limits`/
    /// bound pair each for position, velocity, acceleration, jerk; no
    /// effort limits on this particular message). Still `TryFrom`, not
    /// `From`, per D6's uniform surface.
    fn try_from(wrapped: JointLimitsMsg) -> Result<Self, Self::Error> {
        let msg = wrapped.0;
        Ok(CoreJointLimits {
            joint_name: msg.joint_name,
            has_position_limits: msg.has_position_limits,
            min_position: msg.min_position,
            max_position: msg.max_position,
            has_velocity_limits: msg.has_velocity_limits,
            max_velocity: msg.max_velocity,
            has_acceleration_limits: msg.has_acceleration_limits,
            max_acceleration: msg.max_acceleration,
            has_jerk_limits: msg.has_jerk_limits,
            max_jerk: msg.max_jerk,
        })
    }
}

impl TryFrom<CoreJointLimits> for JointLimitsMsgOut {
    type Error = Error;

    fn try_from(c: CoreJointLimits) -> Result<Self, Self::Error> {
        Ok(JointLimitsMsgOut(moveit_msgs::JointLimits {
            joint_name: c.joint_name,
            has_position_limits: c.has_position_limits,
            min_position: c.min_position,
            max_position: c.max_position,
            has_velocity_limits: c.has_velocity_limits,
            max_velocity: c.max_velocity,
            has_acceleration_limits: c.has_acceleration_limits,
            max_acceleration: c.max_acceleration,
            has_jerk_limits: c.has_jerk_limits,
            max_jerk: c.max_jerk,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_msg() -> moveit_msgs::JointLimits {
        moveit_msgs::JointLimits {
            joint_name: "j1".to_string(),
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            has_velocity_limits: true,
            max_velocity: 2.0,
            has_acceleration_limits: true,
            max_acceleration: 3.0,
            has_jerk_limits: true,
            max_jerk: 4.0,
        }
    }

    #[test]
    fn every_field_converts_msg_to_core() {
        let limits = CoreJointLimits::try_from(JointLimitsMsg(sample_msg())).unwrap();
        assert_eq!(limits.joint_name, "j1");
        assert!(limits.has_position_limits);
        assert_eq!(limits.min_position, -1.0);
        assert_eq!(limits.max_position, 1.0);
        assert!(limits.has_velocity_limits);
        assert_eq!(limits.max_velocity, 2.0);
        assert!(limits.has_acceleration_limits);
        assert_eq!(limits.max_acceleration, 3.0);
        assert!(limits.has_jerk_limits);
        assert_eq!(limits.max_jerk, 4.0);
    }

    #[test]
    fn round_trip_through_msg_drops_nothing() {
        let limits = CoreJointLimits::try_from(JointLimitsMsg(sample_msg())).unwrap();
        let back = JointLimitsMsgOut::try_from(limits).unwrap().0;
        assert_eq!(back.joint_name, "j1");
        assert!(back.has_position_limits);
        assert_eq!(back.min_position, -1.0);
        assert_eq!(back.max_position, 1.0);
        assert!(back.has_velocity_limits);
        assert_eq!(back.max_velocity, 2.0);
        assert!(back.has_acceleration_limits);
        assert_eq!(back.max_acceleration, 3.0);
        assert!(back.has_jerk_limits);
        assert_eq!(back.max_jerk, 4.0);
    }
}
