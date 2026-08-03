// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/src/robot_model.cpp

use std::fmt;

/// Something [`crate::robot_model::RobotModel`] dropped or repaired while
/// building a model from a URDF and SRDF pair.
///
/// Upstream reports every one of these to `rclcpp`'s logger (`RCLCPP_WARN`/
/// `RCLCPP_ERROR`) and returns a model that carries no trace of the decision
/// — the same silent-drop shape `moveit_srdf::Diagnostic` documents for
/// srdfdom, and the same fix applies: each variant here stands for one
/// upstream log call, the model takes the same action upstream takes, and
/// the decision is additionally recorded in
/// [`RobotModel::diagnostics`](crate::robot_model::RobotModel::diagnostics).
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Diagnostic {
    /// A joint's `mimic` names a joint the model does not have. The mimic
    /// relationship is dropped; the joint becomes an ordinary active joint.
    MimicUnknownJoint {
        /// The joint whose `mimic` was dropped.
        joint: String,
        /// The unknown joint name it named.
        mimicked: String,
    },

    /// A joint's `mimic` names a joint with a different variable count. The
    /// mimic relationship is dropped.
    MimicDofMismatch {
        /// The joint whose `mimic` was dropped.
        joint: String,
        /// The joint it tried to mimic.
        mimicked: String,
    },

    /// Mimic joints form a cycle. Every mimic relationship in the model is
    /// cleared (matches upstream: the whole model loses mimic information,
    /// not just the cycle).
    MimicCycle,

    /// Two SRDF groups share a name; the second is dropped.
    DuplicateGroup {
        /// The repeated name.
        group: String,
    },

    /// A group's chains, joints and links resolved to no joints at all. The
    /// group is dropped — matches upstream's "must have at least one valid
    /// joint".
    EmptyGroup {
        /// The name of the dropped group.
        group: String,
    },

    /// A group's subgroups never all resolved (the subgroup itself was
    /// dropped, or named a group that does not exist), so this group could
    /// not be built.
    UnsatisfiedSubgroups {
        /// The name of the group that could not be processed.
        group: String,
    },

    /// A `<joint_property>` named a property this model does not know how to
    /// apply.
    UnknownJointProperty {
        /// The joint the property was attached to.
        joint: String,
        /// The property name.
        property: String,
    },

    /// A `<joint_property>` applies only to a specific joint type, and this
    /// joint is not that type.
    JointPropertyWrongType {
        /// The joint the property was attached to.
        joint: String,
        /// The property name.
        property: String,
        /// This joint's actual type name.
        joint_type: &'static str,
    },

    /// A `<joint_property>` value did not parse as the number it needed to
    /// be.
    JointPropertyMalformedValue {
        /// The joint the property was attached to.
        joint: String,
        /// The property name.
        property: String,
        /// The text as written in the document.
        value: String,
    },
}

impl fmt::Display for Diagnostic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MimicUnknownJoint { joint, mimicked } => {
                write!(f, "joint {joint:?} cannot mimic unknown joint {mimicked:?}")
            }
            Self::MimicDofMismatch { joint, mimicked } => write!(
                f,
                "joint {joint:?} cannot mimic joint {mimicked:?}: different variable count"
            ),
            Self::MimicCycle => {
                f.write_str("cycle found among mimic joints; all mimic joints cleared")
            }
            Self::DuplicateGroup { group } => {
                write!(f, "a group named {group:?} already exists; not adding")
            }
            Self::EmptyGroup { group } => {
                write!(f, "group {group:?} must have at least one valid joint")
            }
            Self::UnsatisfiedSubgroups { group } => write!(
                f,
                "group {group:?} could not be processed due to unmet subgroup dependencies"
            ),
            Self::UnknownJointProperty { joint, property } => {
                write!(f, "unknown joint property {property:?} on joint {joint:?}")
            }
            Self::JointPropertyWrongType {
                joint,
                property,
                joint_type,
            } => write!(
                f,
                "cannot apply property {property:?} to joint {joint:?} of type {joint_type}"
            ),
            Self::JointPropertyMalformedValue {
                joint,
                property,
                value,
            } => write!(
                f,
                "unable to parse property {property:?} on joint {joint:?} as a number: {value:?}"
            ),
        }
    }
}
