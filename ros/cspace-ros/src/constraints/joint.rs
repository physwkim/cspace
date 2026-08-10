// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/JointConstraint` <-> [`cspace_planning::constraints::JointConstraint`].
//! See `doc/message-mapping.md` §4.

use cspace_core::error::Error;
use cspace_core::model::RobotModel;
use cspace_planning::constraints::JointConstraint as CoreJointConstraint;
use r2r::moveit_msgs::msg as moveit_msgs;

/// Wraps the wire message with the `&RobotModel` needed to resolve
/// `joint_name` into a variable index (§4: not a pure function of the
/// message alone).
pub struct JointConstraintMsg<'m> {
    /// Resolves `msg.joint_name` to a variable index.
    pub model: &'m RobotModel,
    /// The wire message, unmodified.
    pub msg: moveit_msgs::JointConstraint,
}

/// Plain local wrapper, for the core->msg direction.
pub struct JointConstraintMsgOut(pub moveit_msgs::JointConstraint);

impl<'m> TryFrom<JointConstraintMsg<'m>> for CoreJointConstraint {
    type Error = Error;

    fn try_from(wrapped: JointConstraintMsg<'m>) -> Result<Self, Self::Error> {
        let JointConstraintMsg { model, msg } = wrapped;
        CoreJointConstraint::new(
            model,
            &msg.joint_name,
            msg.position,
            msg.tolerance_above,
            msg.tolerance_below,
            msg.weight,
        )
    }
}

impl TryFrom<CoreJointConstraint> for JointConstraintMsgOut {
    type Error = Error;

    /// Total: every field on a constructed [`CoreJointConstraint`] is
    /// already valid by construction. `joint_variable_name()` is the exact
    /// wire-form string (`"joint"` or `"joint/local"`) upstream's own
    /// convention expects -- confirmed in `cspace_planning::constraints::joint`'s own
    /// doc comment, not rebuilt from `local_variable_name()` separately.
    fn try_from(c: CoreJointConstraint) -> Result<Self, Self::Error> {
        Ok(JointConstraintMsgOut(moveit_msgs::JointConstraint {
            joint_name: c.joint_variable_name().to_string(),
            position: c.desired_joint_position(),
            tolerance_above: c.joint_tolerance_above(),
            tolerance_below: c.joint_tolerance_below(),
            weight: c.weight(),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;

    #[test]
    fn converts_and_resolves_against_model() {
        let model = one_joint_model();
        let msg = moveit_msgs::JointConstraint {
            joint_name: "j1".to_string(),
            position: 0.2,
            tolerance_above: 0.1,
            tolerance_below: 0.1,
            weight: 1.0,
        };
        let c = CoreJointConstraint::try_from(JointConstraintMsg { model: &model, msg }).unwrap();
        assert_eq!(c.desired_joint_position(), 0.2);
    }

    #[test]
    fn unknown_joint_name_is_rejected() {
        let model = one_joint_model();
        let msg = moveit_msgs::JointConstraint {
            joint_name: "no_such_joint".to_string(),
            position: 0.0,
            tolerance_above: 0.1,
            tolerance_below: 0.1,
            weight: 1.0,
        };
        let err =
            CoreJointConstraint::try_from(JointConstraintMsg { model: &model, msg }).unwrap_err();
        // `kind`, not just the variant: `JointConstraint::new` has a second
        // `UnknownName` site (an unresolved local variable name on a
        // multi-DOF `"joint/local"` name) with `kind: "variable"` -- only
        // the field tells this test apart from that sibling.
        assert!(
            matches!(&err, Error::UnknownName { kind, .. } if *kind == "joint"),
            "got: {err:?}"
        );
    }

    #[test]
    fn round_trip_through_msg() {
        let model = one_joint_model();
        let msg = moveit_msgs::JointConstraint {
            joint_name: "j1".to_string(),
            position: 0.3,
            tolerance_above: 0.05,
            tolerance_below: 0.05,
            weight: 2.0,
        };
        let c = CoreJointConstraint::try_from(JointConstraintMsg { model: &model, msg }).unwrap();
        let back = JointConstraintMsgOut::try_from(c).unwrap().0;
        assert_eq!(back.joint_name, "j1");
        assert_eq!(back.position, 0.3);
        assert_eq!(back.weight, 2.0);
    }
}
