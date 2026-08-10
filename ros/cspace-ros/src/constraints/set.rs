// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/Constraints` <-> [`cspace_constraints::KinematicConstraintSet`].
//! See `doc/message-mapping.md` §3.
//!
//! `Constraints.name` has no [`KinematicConstraintSet`] counterpart (the
//! core type is a bare `Vec<Constraint>`, see that type's own doc comment
//! on why it dropped upstream's parallel-vector/name bookkeeping) -- msg->core
//! drops it (documented, not silently: see the `name` field handling below),
//! core->msg emits `String::new()`. Re-checked round 5 against
//! `crates/cspace-constraints/src/set.rs:47-49` -- `KinematicConstraintSet`
//! is still exactly `{ constraints: Vec<Constraint> }`. Expires if it grows
//! a `name` field; `cspace-constraints`'s call, not this crate's.

use cspace_constraints::{Constraint, KinematicConstraintSet};
use cspace_core::error::Error;
use cspace_core::model::RobotModel;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::joint::{JointConstraintMsg, JointConstraintMsgOut};
use super::orientation::{OrientationConstraintMsg, OrientationConstraintMsgOut};
use super::position::{PositionConstraintMsg, PositionConstraintMsgOut};
use super::visibility::{VisibilityConstraintMsg, VisibilityConstraintMsgOut};

/// Wraps the wire message with the `&RobotModel` needed by every element
/// conversion (§3).
pub struct ConstraintsMsg<'m> {
    /// Passed through to every element conversion.
    pub model: &'m RobotModel,
    /// The wire message, unmodified.
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
            let c = cspace_constraints::JointConstraint::try_from(JointConstraintMsg {
                model,
                msg: joint_msg,
            })?;
            set.push(Constraint::Joint(c));
        }
        for position_msg in msg.position_constraints {
            let c = cspace_constraints::PositionConstraint::try_from(PositionConstraintMsg {
                model,
                msg: position_msg,
            })?;
            set.push(Constraint::Position(c));
        }
        for orientation_msg in msg.orientation_constraints {
            let c =
                cspace_constraints::OrientationConstraint::try_from(OrientationConstraintMsg {
                    model,
                    msg: orientation_msg,
                })?;
            set.push(Constraint::Orientation(c));
        }
        for visibility_msg in msg.visibility_constraints {
            let c = cspace_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
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
        // `kind`, not just the variant (same as `joint.rs`'s
        // `unknown_joint_name_is_rejected`): only `joint_constraints` is
        // populated here, so the sole reachable `UnknownName` site is
        // `JointConstraint::new`'s joint-lookup (`kind: "joint"`), not its
        // sibling local-variable site (`kind: "variable"`).
        assert!(
            matches!(&err, Error::UnknownName { kind, .. } if *kind == "joint"),
            "got: {err:?}"
        );
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

    // PORTING-PLAN.md §199 (D14) + §205 (tripwire, not `#[ignore]`): a
    // `weight` of `0.0` is not an unanswerable lookup (D6's actual scope --
    // an unresolved frame name) but a wire default upstream's own
    // constructors give meaning to -- `kinematic_constraint.cpp:263`
    // (`JointConstraint`) and the matching `PositionConstraint`/
    // `OrientationConstraint`/`VisibilityConstraint` lines the coordinator
    // cited (`:450`/`:641`/`:871`) all warn and substitute `1.0` instead of
    // failing. `crates/cspace-constraints`'s four constructors currently
    // reject it with `Err` instead (`weight <= EPS`, one site per type,
    // confirmed present in all four by reading each source file this
    // round, not just `JointConstraint`) -- that crate owns the D14 fix,
    // not this one.
    //
    // The tripwire fired. These four tests were written asserting the
    // CURRENT (wrong) `Err` behavior precisely so they would go red the
    // moment D14 landed rather than sit `#[ignore]`d and silently green
    // either way (§184/§197.3, the shape this session closed twice). D14
    // landed in `551b719`; all four went red on the first merge gate, and
    // each `Ok` value carried `weight: 1.0`. They now assert that value --
    // which is what makes them a wire-path check and not a duplicate of
    // `crates/cspace-constraints`'s own boundary tests: nothing else covers
    // a `weight` field that arrives `0.0` because a publisher never set it,
    // travelling the whole `TryFrom` chain into the constructor.
    #[test]
    fn unspecified_joint_weight_is_normalized_to_one_not_rejected() {
        let model = one_joint_model();
        let mut unspecified_weight = joint_msg("j1");
        unspecified_weight.weight = 0.0; // wire default: never set by the publisher
        let msg = moveit_msgs::Constraints {
            name: "unspecified_weight".to_string(),
            joint_constraints: vec![unspecified_weight],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg })
            .expect("D14: an unset wire `weight` normalizes to 1.0, it does not reject");
        match &set.constraints()[0] {
            Constraint::Joint(c) => assert_eq!(c.weight(), 1.0),
            other => panic!("expected Joint, got {other:?}"),
        }
    }

    fn identity_pose() -> r2r::geometry_msgs::msg::Pose {
        r2r::geometry_msgs::msg::Pose {
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
        }
    }

    #[test]
    fn unspecified_position_weight_is_normalized_to_one_not_rejected() {
        let model = one_joint_model();
        let msg = moveit_msgs::Constraints {
            name: "unspecified_weight".to_string(),
            joint_constraints: vec![],
            position_constraints: vec![moveit_msgs::PositionConstraint {
                header: r2r::std_msgs::msg::Header {
                    frame_id: model.model_frame().to_string(),
                    ..Default::default()
                },
                link_name: "tip".to_string(),
                target_point_offset: Default::default(),
                constraint_region: moveit_msgs::BoundingVolume {
                    primitives: vec![r2r::shape_msgs::msg::SolidPrimitive {
                        type_: 2, // SPHERE
                        dimensions: vec![0.05],
                        polygon: Default::default(),
                    }],
                    primitive_poses: vec![identity_pose()],
                    meshes: vec![],
                    mesh_poses: vec![],
                },
                weight: 0.0, // wire default: never set by the publisher
            }],
            orientation_constraints: vec![],
            visibility_constraints: vec![],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg })
            .expect("D14: an unset wire `weight` normalizes to 1.0, it does not reject");
        match &set.constraints()[0] {
            Constraint::Position(c) => assert_eq!(c.weight(), 1.0),
            other => panic!("expected Position, got {other:?}"),
        }
    }

    #[test]
    fn unspecified_orientation_weight_is_normalized_to_one_not_rejected() {
        let model = one_joint_model();
        let msg = moveit_msgs::Constraints {
            name: "unspecified_weight".to_string(),
            joint_constraints: vec![],
            position_constraints: vec![],
            orientation_constraints: vec![moveit_msgs::OrientationConstraint {
                header: r2r::std_msgs::msg::Header {
                    frame_id: model.model_frame().to_string(),
                    ..Default::default()
                },
                orientation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                link_name: "tip".to_string(),
                absolute_x_axis_tolerance: 0.1,
                absolute_y_axis_tolerance: 0.1,
                absolute_z_axis_tolerance: 0.1,
                parameterization: 0, // XYZ_EULER_ANGLES
                weight: 0.0,         // wire default: never set by the publisher
            }],
            visibility_constraints: vec![],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg })
            .expect("D14: an unset wire `weight` normalizes to 1.0, it does not reject");
        match &set.constraints()[0] {
            Constraint::Orientation(c) => assert_eq!(c.weight(), 1.0),
            other => panic!("expected Orientation, got {other:?}"),
        }
    }

    #[test]
    fn unspecified_visibility_weight_is_normalized_to_one_not_rejected() {
        let model = one_joint_model();
        let identity = identity_pose();
        let msg = moveit_msgs::Constraints {
            name: "unspecified_weight".to_string(),
            joint_constraints: vec![],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![moveit_msgs::VisibilityConstraint {
                target_radius: 0.1,
                target_pose: r2r::geometry_msgs::msg::PoseStamped {
                    header: r2r::std_msgs::msg::Header {
                        frame_id: model.model_frame().to_string(),
                        ..Default::default()
                    },
                    pose: identity.clone(),
                },
                cone_sides: 4,
                sensor_pose: r2r::geometry_msgs::msg::PoseStamped {
                    header: r2r::std_msgs::msg::Header {
                        frame_id: "tip".to_string(),
                        ..Default::default()
                    },
                    pose: identity,
                },
                max_view_angle: 0.0,
                max_range_angle: 0.0,
                sensor_view_direction: 2,
                weight: 0.0, // wire default: never set by the publisher
            }],
        };
        let set = KinematicConstraintSet::try_from(ConstraintsMsg { model: &model, msg })
            .expect("D14: an unset wire `weight` normalizes to 1.0, it does not reject");
        match &set.constraints()[0] {
            Constraint::Visibility(c) => assert_eq!(c.weight(), 1.0),
            other => panic!("expected Visibility, got {other:?}"),
        }
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
