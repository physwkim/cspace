// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/VisibilityConstraint` <-> [`moveit_constraints::VisibilityConstraint`].
//! See `doc/message-mapping.md` §7 -- including the `sensor_view_direction`
//! landmine this module exists specifically to close (mandatory this round).
//!
//! # core->msg, round 5: EXPIRED, now implemented
//!
//! Through round 4 this module's doc comment said core->msg could not be
//! written because `moveit_constraints::VisibilityConstraint` exposed only
//! `sensor_frame()`, `target_frame()`, `cone_sides()`, `enabled()`. Round 5's
//! 72-commit-drift re-audit (`doc/message-mapping.md`'s top note) re-checked
//! that claim against current `crates/moveit-constraints/src/visibility.rs`
//! instead of trusting it: the accessor list requested from that crate's
//! owner already landed -- `sensor()`, `target()`, `sensor_view_direction()`,
//! `target_radius()`, `max_view_angle()`, `max_range_angle()`, `weight()` are
//! all present now. `TryFrom<VisibilityConstraint> for
//! VisibilityConstraintMsgOut` below is the fix.

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
    /// Resolves `msg.sensor_pose`'s and `msg.target_pose`'s `header.frame_id`.
    pub model: &'m RobotModel,
    /// The wire message, unmodified.
    pub msg: moveit_msgs::VisibilityConstraint,
}

/// Plain local wrapper, for the core->msg direction.
pub struct VisibilityConstraintMsgOut(pub moveit_msgs::VisibilityConstraint);

impl<'m> TryFrom<VisibilityConstraintMsg<'m>> for moveit_constraints::VisibilityConstraint {
    type Error = Error;

    fn try_from(wrapped: VisibilityConstraintMsg<'m>) -> Result<Self, Self::Error> {
        let VisibilityConstraintMsg { model, msg } = wrapped;
        let tf = minimal_transforms(model)?;

        // Round 14 (`kinematic_constraint.cpp:818-829`): upstream's own
        // guard is `if (vc.cone_sides < 3)`, which clamps *any* value below
        // 3 -- negative included -- up to 3, and never fails. That is D14's
        // shape, not D6's: `cone_sides_` (`unsigned int`) is only ever
        // assigned from `vc.cone_sides` in the `>= 3` branch, so upstream's
        // own guard order already prevents an `int32 -> unsigned` wraparound
        // on a negative value -- there is no "unresolvable input" here, just
        // a floor upstream itself defines. Rejecting `cone_sides < 0`
        // outright (this crate's previous behavior, before this round) was
        // stricter than upstream *and* than this crate's own
        // `VisibilityConstraint::new`, which already clamps 0/1/2 up to 3
        // (its own doc comment: "a real geometric floor, not a sentinel to
        // repair") -- wire-only strictness with no invariant behind it.
        // `.max(0)` before the cast reorders this crate's guard to match
        // upstream's: it removes the wraparound risk (`as usize` on a
        // non-negative `i32` is always exact) without rejecting anything
        // `VisibilityConstraint::new`'s own clamp would accept anyway.
        let cone_sides = msg.cone_sides.max(0) as usize;

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

impl TryFrom<moveit_constraints::VisibilityConstraint> for VisibilityConstraintMsgOut {
    type Error = Error;

    /// Total: every accessor this needs is infallible, and `Pose`'s
    /// `TryFrom<Isometry3>` is total (see `geometry.rs`). `target_radius`/
    /// `max_view_angle`/`max_range_angle` map `None` back to wire `0.0`,
    /// the reverse of msg->core's `normalize_criterion` (near-zero-or-empty
    /// treated as unconstrained on the way in).
    fn try_from(c: moveit_constraints::VisibilityConstraint) -> Result<Self, Self::Error> {
        let sensor_view_direction = SensorViewDirectionMsg::try_from(c.sensor_view_direction())?.0;
        Ok(VisibilityConstraintMsgOut(
            moveit_msgs::VisibilityConstraint {
                target_radius: c.target_radius().unwrap_or(0.0),
                target_pose: r2r::geometry_msgs::msg::PoseStamped {
                    header: r2r::std_msgs::msg::Header {
                        frame_id: c.target_frame().to_string(),
                        ..Default::default()
                    },
                    pose: Pose::try_from(c.target())?.0,
                },
                cone_sides: c.cone_sides() as i32,
                sensor_pose: r2r::geometry_msgs::msg::PoseStamped {
                    header: r2r::std_msgs::msg::Header {
                        frame_id: c.sensor_frame().to_string(),
                        ..Default::default()
                    },
                    pose: Pose::try_from(c.sensor())?.0,
                },
                max_view_angle: c.max_view_angle().unwrap_or(0.0),
                max_range_angle: c.max_range_angle().unwrap_or(0.0),
                sensor_view_direction,
                weight: c.weight(),
            },
        ))
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
    fn round_trip_through_msg() {
        // Every numeric field gets a distinct value so a mixed-up accessor
        // (e.g. target_radius <-> max_view_angle, or sensor_pose <->
        // target_pose) fails this test instead of hiding behind a repeated
        // constant -- same discipline as c8dd883's start_state round-trip.
        let model = one_joint_model();
        let sensor_pose = r2r::geometry_msgs::msg::Pose {
            position: r2r::geometry_msgs::msg::Point {
                x: 1.0,
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
        let target_pose = r2r::geometry_msgs::msg::Pose {
            position: r2r::geometry_msgs::msg::Point {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
            orientation: r2r::geometry_msgs::msg::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let msg = moveit_msgs::VisibilityConstraint {
            target_radius: 0.2,
            target_pose: r2r::geometry_msgs::msg::PoseStamped {
                header: r2r::std_msgs::msg::Header {
                    frame_id: model.model_frame().to_string(),
                    ..Default::default()
                },
                pose: target_pose,
            },
            cone_sides: 5,
            sensor_pose: r2r::geometry_msgs::msg::PoseStamped {
                header: r2r::std_msgs::msg::Header {
                    frame_id: "tip".to_string(),
                    ..Default::default()
                },
                pose: sensor_pose,
            },
            max_view_angle: 0.3,
            max_range_angle: 0.4,
            sensor_view_direction: 1, // SENSOR_Y
            weight: 0.6,
        };
        let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap();
        let back = VisibilityConstraintMsgOut::try_from(c).unwrap().0;
        assert_eq!(back.target_radius, 0.2);
        assert_eq!(back.max_view_angle, 0.3);
        assert_eq!(back.max_range_angle, 0.4);
        assert_eq!(back.cone_sides, 5);
        assert_eq!(back.weight, 0.6);
        assert_eq!(back.sensor_view_direction, 1);
        assert_eq!(back.sensor_pose.header.frame_id, "tip");
        assert_eq!(back.sensor_pose.pose.position.x, 1.0);
        assert_eq!(back.sensor_pose.pose.position.y, 0.0);
        assert_eq!(back.target_pose.header.frame_id, model.model_frame());
        assert_eq!(back.target_pose.pose.position.x, 0.0);
        assert_eq!(back.target_pose.pose.position.y, 2.0);
    }

    #[test]
    fn negative_cone_sides_is_clamped_to_three_not_rejected() {
        // Round 14: matches kinematic_constraint.cpp:818-829's own guard
        // order -- upstream clamps any cone_sides < 3 (negative included)
        // rather than failing. See TryFrom<VisibilityConstraintMsg>'s own
        // comment for why rejecting was stricter than both upstream and
        // VisibilityConstraint::new's own clamp.
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.cone_sides = -1;
        let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap();
        assert_eq!(c.cone_sides(), 3);
    }

    #[test]
    fn negative_target_radius_activates_but_negative_angles_stay_inactive() {
        // `0ca8916` split moveit-constraints's own normalization: negative
        // target_radius activates at its magnitude (kinematic_constraint.cpp:818,
        // fabs() before the >eps gate), while a negative max_view_angle/
        // max_range_angle fails that gate and stays inactive (`:879-880`, no
        // fabs()). This crate's wire mapping passes all three straight
        // through with no `.abs()` anywhere (see `Some(msg.target_radius)`
        // etc. above) -- confirming the asymmetry survives the wire
        // boundary, not just moveit-constraints's own unit tests.
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.target_radius = -0.5;
        msg.max_view_angle = -0.5;
        msg.max_range_angle = -0.5;
        let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap();
        assert_eq!(c.target_radius(), Some(0.5));
        assert_eq!(c.max_view_angle(), None);
        assert_eq!(c.max_range_angle(), None);
    }

    #[test]
    fn i32_min_cone_sides_does_not_wrap_around_when_cast() {
        // Boundary of the wraparound risk `.max(0)` exists to prevent: a
        // naive `as usize` on i32::MIN would not merely be wrong, it would
        // silently become a huge positive count instead of clamping to 3.
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.cone_sides = i32::MIN;
        let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap();
        assert_eq!(c.cone_sides(), 3);
    }

    #[test]
    fn sensor_and_target_pose_with_norm_2_orientation_succeed_and_normalize() {
        // PORTING-PLAN.md §215's per-site table: `sensor_pose`/`target_pose`
        // at :114-115 share the generic Pose rule with the other eight
        // sites -- confirmed through this site's own full call chain, not
        // just the bare conversion in geometry.rs's own tests.
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.sensor_pose.pose.orientation = r2r::geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 2.0,
        };
        msg.target_pose.pose.orientation = r2r::geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 2.0,
        };
        let c = moveit_constraints::VisibilityConstraint::try_from(VisibilityConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap();
        let sensor_norm = c.sensor().rotation.into_inner().norm();
        let target_norm = c.target().rotation.into_inner().norm();
        assert!((sensor_norm - 1.0).abs() < 1e-12, "got: {sensor_norm}");
        assert!((target_norm - 1.0).abs() < 1e-12, "got: {target_norm}");
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
