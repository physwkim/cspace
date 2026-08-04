// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/Constraints` <-> [`moveit_constraints::KinematicConstraintSet`].
//! See `doc/message-mapping.md` §3.
//!
//! `Constraints.name` has no [`KinematicConstraintSet`] counterpart (the
//! core type is a bare `Vec<Constraint>`, see that type's own doc comment
//! on why it dropped upstream's parallel-vector/name bookkeeping) -- msg->core
//! drops it (documented, not silently: see the `name` field handling below),
//! core->msg emits `String::new()`. Re-checked round 5 against
//! `crates/moveit-constraints/src/set.rs:47-49` -- `KinematicConstraintSet`
//! is still exactly `{ constraints: Vec<Constraint> }`. Expires if it grows
//! a `name` field; `moveit-constraints`'s call, not this crate's.

use moveit_constraints::{Constraint, KinematicConstraintSet};
use moveit_error::Error;
use moveit_model::RobotModel;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::joint::{JointConstraintMsg, JointConstraintMsgOut};
use super::orientation::{OrientationConstraintMsg, OrientationConstraintMsgOut};
use super::position::{PositionConstraintMsg, PositionConstraintMsgOut};
use super::visibility::{VisibilityConstraintMsg, VisibilityConstraintMsgOut};

/// Wraps the wire message with the `&RobotModel` needed by every element
/// conversion (§3).
pub struct ConstraintsMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: moveit_msgs::Constraints,
}

/// Plain local wrapper, for the core->msg direction.
#[derive(Debug)]
pub struct ConstraintsMsgOut(pub moveit_msgs::Constraints);

impl<'m> TryFrom<ConstraintsMsg<'m>> for KinematicConstraintSet {
    type Error = Error;

    /// Any single element failing fails the whole conversion -- a
    /// `Constraints` message is "all must be satisfied," so a constraint
    /// this port cannot faithfully represent must not be silently dropped
    /// from the set (that would make the set easier to satisfy than the
    /// message actually asked for).
    fn try_from(wrapped: ConstraintsMsg<'m>) -> Result<Self, Self::Error> {
        let ConstraintsMsg { model, msg } = wrapped;
        let mut set = KinematicConstraintSet::new();

        for joint_msg in msg.joint_constraints {
            let c = moveit_constraints::JointConstraint::try_from(JointConstraintMsg {
                model,
                msg: joint_msg,
            })?;
            set.push(Constraint::Joint(c));
        }
        for position_msg in msg.position_constraints {
            let c = moveit_constraints::PositionConstraint::try_from(PositionConstraintMsg {
                model,
                msg: position_msg,
            })?;
            set.push(Constraint::Position(c));
        }
        for orientation_msg in msg.orientation_constraints {
            let c =
                moveit_constraints::OrientationConstraint::try_from(OrientationConstraintMsg {
                    model,
                    msg: orientation_msg,
                })?;
            set.push(Constraint::Orientation(c));
        }
        for visibility_msg in msg.visibility_constraints {
            let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
                model,
                msg: visibility_msg,
            })?;
            set.push(Constraint::Visibility(c));
        }

        Ok(set)
    }
}

impl TryFrom<KinematicConstraintSet> for ConstraintsMsgOut {
    type Error = Error;

    /// **[R5, EXPIRED]** Through round 4 this rejected any
    /// [`Constraint::Visibility`] member (`VisibilityConstraint` had no
    /// core->msg conversion -- see `visibility.rs`'s module doc comment for
    /// the accessor list that has since landed). Every variant is now total.
    fn try_from(set: KinematicConstraintSet) -> Result<Self, Self::Error> {
        let mut out = moveit_msgs::Constraints {
            name: String::new(),
            joint_constraints: Vec::new(),
            position_constraints: Vec::new(),
            orientation_constraints: Vec::new(),
            visibility_constraints: Vec::new(),
        };
        for constraint in set.constraints().iter().cloned() {
            match constraint {
                Constraint::Joint(c) => {
                    out.joint_constraints
                        .push(JointConstraintMsgOut::try_from(c)?.0);
                }
                Constraint::Position(c) => {
                    out.position_constraints
                        .push(PositionConstraintMsgOut::try_from(c)?.0);
                }
                Constraint::Orientation(c) => {
                    out.orientation_constraints
                        .push(OrientationConstraintMsgOut::try_from(c)?.0);
                }
                Constraint::Visibility(c) => {
                    out.visibility_constraints
                        .push(VisibilityConstraintMsgOut::try_from(c)?.0);
                }
            }
        }
        Ok(ConstraintsMsgOut(out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;

    fn joint_msg(name: &str) -> moveit_msgs::JointConstraint {
        moveit_msgs::JointConstraint {
            joint_name: name.to_string(),
            position: 0.1,
            tolerance_above: 0.1,
            tolerance_below: 0.1,
            weight: 1.0,
        }
    }

    #[test]
    fn empty_constraints_is_empty_set() {
        let model = one_joint_model();
        let msg = moveit_msgs::Constraints {
            name: "empty".to_string(),
            joint_constraints: vec![],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg }).unwrap();
        assert!(set.is_empty());
    }

    #[test]
    fn aggregates_joint_constraints() {
        let model = one_joint_model();
        let msg = moveit_msgs::Constraints {
            name: "one_joint".to_string(),
            joint_constraints: vec![joint_msg("j1")],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg }).unwrap();
        assert_eq!(set.len(), 1);
        assert!(matches!(set.constraints()[0], Constraint::Joint(_)));
    }

    #[test]
    fn one_bad_element_fails_the_whole_conversion() {
        let model = one_joint_model();
        let msg = moveit_msgs::Constraints {
            name: "bad".to_string(),
            joint_constraints: vec![joint_msg("j1"), joint_msg("no_such_joint")],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![],
        };
        let err =
            KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::UnknownName { .. }), "got: {err:?}");
    }

    #[test]
    fn round_trip_through_msg() {
        let model = one_joint_model();
        let msg = moveit_msgs::Constraints {
            name: "one_joint".to_string(),
            joint_constraints: vec![joint_msg("j1")],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg }).unwrap();
        let back = ConstraintsMsgOut::try_from(set).unwrap().0;
        assert_eq!(back.joint_constraints.len(), 1);
        assert_eq!(back.joint_constraints[0].joint_name, "j1");
    }

    #[test]
    fn visibility_member_round_trips() {
        let model = one_joint_model();
        let identity_pose = r2r::geometry_msgs::msg::Pose {
            position: r2r::geometry_msgs::msg::Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: r2r::geometry_msgs::msg::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let visibility_msg = moveit_msgs::VisibilityConstraint {
            target_radius: 0.1,
            target_pose: r2r::geometry_msgs::msg::PoseStamped {
                header: r2r::std_msgs::msg::Header {
                    frame_id: model.model_frame().to_string(),
                    ..Default::default()
                },
                pose: identity_pose.clone(),
            },
            cone_sides: 4,
            sensor_pose: r2r::geometry_msgs::msg::PoseStamped {
                header: r2r::std_msgs::msg::Header {
                    frame_id: "tip".to_string(),
                    ..Default::default()
                },
                pose: identity_pose,
            },
            max_view_angle: 0.0,
            max_range_angle: 0.0,
            sensor_view_direction: 2,
            weight: 1.0,
        };
        let msg = moveit_msgs::Constraints {
            name: "has_visibility".to_string(),
            joint_constraints: vec![],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![visibility_msg],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg }).unwrap();
        assert!(matches!(set.constraints()[0], Constraint::Visibility(_)));
        let back = ConstraintsMsgOut::try_from(set).unwrap().0;
        assert_eq!(back.visibility_constraints.len(), 1);
        assert_eq!(back.visibility_constraints[0].cone_sides, 4);
        assert_eq!(back.visibility_constraints[0].target_radius, 0.1);
    }
}
