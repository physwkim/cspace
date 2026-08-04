// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/VisibilityConstraint` <-> [`moveit_constraints::VisibilityConstraint`].
//! See `doc/message-mapping.md` §7 -- including the `sensor_view_direction`
//! landmine this module exists specifically to close (mandatory this round).
//!
//! # core->msg is not implemented this round -- missing accessors, not a
//! # design decision
//!
//! `moveit_constraints::VisibilityConstraint`'s public API
//! (`crates/moveit-constraints/src/visibility.rs`) exposes `sensor_frame()`,
//! `target_frame()`, `cone_sides()`, `enabled()` -- and nothing else. There
//! is no accessor for `weight`, `sensor_view_direction`, `target_radius`/
//! `max_view_angle`/`max_range_angle`, or the sensor/target poses (`FramedPose`
//! and its `pose` field are private to that module). A `TryFrom<VisibilityConstraint>
//! for VisibilityConstraintMsgOut` cannot be written against the crate's
//! current public surface -- not "hasn't been done," genuinely can't be, short
//! of adding accessors to `moveit-constraints` (not this crate's to edit; see
//! this round's report for the exact accessor list requested from that
//! crate's owner). msg->core is fully implemented below.

use moveit_constraints::{SensorSpec, SensorViewDirection, TargetSpec, VisibilityCriteria};
use moveit_error::Error;
use moveit_geometry::Isometry3;
use moveit_model::RobotModel;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::context::minimal_transforms;
use crate::geometry::Pose;

/// Wraps the wire `sensor_view_direction: u8` value. Bidirectional and
/// standalone (not folded into the wrapper below) so it can be tested in
/// isolation -- this is the mandatory landmine fix: the core enum's
/// *declared* variant order is `SensorX, SensorY, SensorZ`, but the *wire*
/// encoding is the reverse (`SENSOR_Z=0, SENSOR_Y=1, SENSOR_X=2`,
/// confirmed against `moveit_msgs/msg/VisibilityConstraint.msg`'s own
/// constants and `moveit_constraints::visibility`'s own `axis_column()` doc
/// comment, "upstream indexes this as `col(2 - sensor_view_direction_)`").
/// A conversion written by positional/derived-discriminant cast (e.g.
/// `[SensorX, SensorY, SensorZ][val as usize]`, or `unsafe { transmute }`)
/// would silently swap X and Z -- this match is written against the three
/// named wire constants explicitly, never positionally.
pub struct SensorViewDirectionMsg(pub u8);

const SENSOR_Z: u8 = 0;
const SENSOR_Y: u8 = 1;
const SENSOR_X: u8 = 2;

impl TryFrom<SensorViewDirectionMsg> for SensorViewDirection {
    type Error = Error;

    fn try_from(msg: SensorViewDirectionMsg) -> Result<Self, Self::Error> {
        match msg.0 {
            SENSOR_Z => Ok(SensorViewDirection::SensorZ),
            SENSOR_Y => Ok(SensorViewDirection::SensorY),
            SENSOR_X => Ok(SensorViewDirection::SensorX),
            other => Err(Error::construct(format!(
                "VisibilityConstraint.sensor_view_direction={other} is none \
                 of SENSOR_Z(0)/SENSOR_Y(1)/SENSOR_X(2)"
            ))),
        }
    }
}

impl TryFrom<SensorViewDirection> for SensorViewDirectionMsg {
    type Error = Error;

    fn try_from(dir: SensorViewDirection) -> Result<Self, Self::Error> {
        Ok(SensorViewDirectionMsg(match dir {
            SensorViewDirection::SensorZ => SENSOR_Z,
            SensorViewDirection::SensorY => SENSOR_Y,
            SensorViewDirection::SensorX => SENSOR_X,
        }))
    }
}

/// Wraps the wire message with the `&RobotModel` needed to resolve
/// `sensor_pose`/`target_pose`'s `header.frame_id` (§7).
pub struct VisibilityConstraintMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: moveit_msgs::VisibilityConstraint,
}

impl<'m> TryFrom<VisibilityConstraintMsg<'m>> for moveit_constraints::VisibilityConstraint {
    type Error = Error;

    fn try_from(wrapped: VisibilityConstraintMsg<'m>) -> Result<Self, Self::Error> {
        let VisibilityConstraintMsg { model, msg } = wrapped;
        let tf = minimal_transforms(model)?;

        // `cone_sides < 0` before the `i32 -> usize` cast: a naive `as usize`
        // on a negative value silently becomes a huge number (D6's "failure
        // becomes a silent default", the exact bug class this guards). Note
        // this is a *different* guard from `cone_sides < 3`: the crate's own
        // `VisibilityConstraint::new` already clamps 0/1/2 up to 3 by design
        // (its own doc comment: "a real geometric floor, not a sentinel to
        // repair") -- message-mapping.md §7's round-1 note that this
        // `TryFrom` "must reject cone_sides < 3" was written before
        // `moveit_msgs` was in the image to check against; corrected here.
        if msg.cone_sides < 0 {
            return Err(Error::construct(format!(
                "VisibilityConstraint.cone_sides={} is negative",
                msg.cone_sides
            )));
        }
        let cone_sides = msg.cone_sides as usize;

        let view_direction =
            SensorViewDirection::try_from(SensorViewDirectionMsg(msg.sensor_view_direction))?;
        let sensor_pose = Isometry3::try_from(Pose(msg.sensor_pose.pose))?;
        let target_pose = Isometry3::try_from(Pose(msg.target_pose.pose))?;

        moveit_constraints::VisibilityConstraint::new(
            model,
            &tf,
            SensorSpec {
                frame_id: &msg.sensor_pose.header.frame_id,
                pose: sensor_pose,
                view_direction,
            },
            TargetSpec {
                frame_id: &msg.target_pose.header.frame_id,
                pose: target_pose,
            },
            cone_sides,
            VisibilityCriteria {
                target_radius: Some(msg.target_radius),
                max_view_angle: Some(msg.max_view_angle),
                max_range_angle: Some(msg.max_range_angle),
            },
            msg.weight,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;

    #[test]
    fn sensor_view_direction_matches_wire_constants_not_position() {
        // The landmine: wire SENSOR_Z=0/SENSOR_Y=1/SENSOR_X=2 is the reverse
        // of the core enum's declared order (SensorX, SensorY, SensorZ). A
        // positional cast (`[SensorX, SensorY, SensorZ][0]`) would give
        // SensorX for wire value 0; the correct answer is SensorZ.
        assert_eq!(
            SensorViewDirection::try_from(SensorViewDirectionMsg(0)).unwrap(),
            SensorViewDirection::SensorZ
        );
        assert_eq!(
            SensorViewDirection::try_from(SensorViewDirectionMsg(1)).unwrap(),
            SensorViewDirection::SensorY
        );
        assert_eq!(
            SensorViewDirection::try_from(SensorViewDirectionMsg(2)).unwrap(),
            SensorViewDirection::SensorX
        );
    }

    #[test]
    fn sensor_view_direction_round_trips() {
        for dir in [
            SensorViewDirection::SensorX,
            SensorViewDirection::SensorY,
            SensorViewDirection::SensorZ,
        ] {
            let wire = SensorViewDirectionMsg::try_from(dir).unwrap();
            let back = SensorViewDirection::try_from(wire).unwrap();
            assert_eq!(back, dir);
        }
    }

    #[test]
    fn invalid_sensor_view_direction_is_rejected() {
        let err = SensorViewDirection::try_from(SensorViewDirectionMsg(3)).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    fn valid_msg(model: &RobotModel) -> moveit_msgs::VisibilityConstraint {
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
        moveit_msgs::VisibilityConstraint {
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
        }
    }

    #[test]
    fn converts_with_model_context() {
        let model = one_joint_model();
        let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg: valid_msg(&model),
        })
        .unwrap();
        assert_eq!(c.cone_sides(), 4);
    }

    #[test]
    fn negative_cone_sides_is_rejected() {
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.cone_sides = -1;
        let err = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn cone_sides_below_3_is_clamped_not_rejected() {
        // Confirms moveit_constraints::VisibilityConstraint::new's own
        // clamp-to-3 behavior, not an error -- see this module's doc
        // comment correcting round 1's stale assumption.
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.cone_sides = 1;
        let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap();
        assert_eq!(c.cone_sides(), 3);
    }
}
