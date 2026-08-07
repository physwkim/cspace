// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/OrientationConstraint` <-> [`moveit_constraints::OrientationConstraint`].
//! See `doc/message-mapping.md` §6.

use moveit_constraints::{
    OrientationConstraint as CoreOrientationConstraint, OrientationTolerance,
};
use moveit_error::Error;
use moveit_geometry::UnitQuaternion;
use moveit_model::RobotModel;
use r2r::moveit_msgs::msg as moveit_msgs;

use super::context::minimal_transforms;
use crate::geometry::{OrientationConstraintQuaternion, Quaternion};

const XYZ_EULER_ANGLES: u8 = 0;
const ROTATION_VECTOR: u8 = 1;

/// Wraps the wire message with the `&RobotModel` needed to resolve
/// `link_name`/`header.frame_id` (§6).
pub struct OrientationConstraintMsg<'m> {
    /// Resolves `msg.link_name` and `msg.header.frame_id`.
    pub model: &'m RobotModel,
    /// The wire message, unmodified.
    pub msg: moveit_msgs::OrientationConstraint,
}

/// Plain local wrapper, for the core->msg direction.
pub struct OrientationConstraintMsgOut(pub moveit_msgs::OrientationConstraint);

/// `parameterization` (0/1) + the three `absolute_*_axis_tolerance` fields
/// -> [`OrientationTolerance`]. Any value other than the two the message's
/// own comment documents (0/1) must be rejected explicitly, not coerced
/// positionally (§6) -- this is the same "don't silently absorb an invalid
/// discriminant" case as `SensorViewDirection`, just with an `Err` instead
/// of a value swap as the wrong-code failure mode.
///
/// # Kept D6, round 14 (`kinematic_constraint.cpp:652-659`)
///
/// Upstream substitutes `XYZ_EULER_ANGLES` for any `parameterization_type_`
/// that is neither `XYZ_EULER_ANGLES(0)` nor `ROTATION_VECTOR(1)`. This
/// looks like D14's "upstream defines the wire default's meaning" shape
/// (the same shape that applied to `weight`), but the two differ in one
/// deciding way: `weight`'s wire default (`0.0`, an unset `float64`) is
/// itself the value the whole `weight<=EPS` branch fires on, so D14 was
/// about what an *unset* field means. `parameterization`'s wire default is
/// already `XYZ_EULER_ANGLES=0` (`OrientationConstraint.msg`'s own comment:
/// "(default value)") -- an unset field is already a valid, meaningful
/// enumerant with no fallback needed. The `!= 0 && != 1` branch can only
/// fire for a value 2..=255 that a publisher *deliberately* wrote, which is
/// not "the wire default" reaching this code at all; it is a genuinely
/// invalid, explicit discriminant, the same shape `SensorViewDirection`'s
/// wire encoding already rejects rather than coerces. D6 applies as
/// originally written: reject, matching `invalid_parameterization_is_rejected`
/// below (present since round 2, re-confirmed this round, not new).
fn tolerance_from_wire(
    parameterization: u8,
    x: f64,
    y: f64,
    z: f64,
) -> Result<OrientationTolerance, Error> {
    match parameterization {
        XYZ_EULER_ANGLES => Ok(OrientationTolerance::XyzEuler { x, y, z }),
        ROTATION_VECTOR => Ok(OrientationTolerance::RotationVector { x, y, z }),
        other => Err(Error::construct(format!(
            "OrientationConstraint.parameterization={other} is neither \
             XYZ_EULER_ANGLES(0) nor ROTATION_VECTOR(1)"
        ))),
    }
}

fn tolerance_to_wire(tolerance: OrientationTolerance) -> (u8, f64, f64, f64) {
    match tolerance {
        OrientationTolerance::XyzEuler { x, y, z } => (XYZ_EULER_ANGLES, x, y, z),
        OrientationTolerance::RotationVector { x, y, z } => (ROTATION_VECTOR, x, y, z),
    }
}

impl<'m> TryFrom<OrientationConstraintMsg<'m>> for CoreOrientationConstraint {
    type Error = Error;

    fn try_from(wrapped: OrientationConstraintMsg<'m>) -> Result<Self, Self::Error> {
        let OrientationConstraintMsg { model, msg } = wrapped;
        let tf = minimal_transforms(model)?;
        // Round 15/§211: this field's own upstream rule
        // (`OrientationConstraint::configure`'s 1e-3 suspicion threshold,
        // `kinematic_constraint.cpp:609-615`) is stricter than -- and
        // different from -- the generic Pose rule every other Quaternion in
        // this crate reaches. `OrientationConstraintQuaternion` names that
        // difference at the type level instead of leaving the caller to
        // pick the right threshold implicitly.
        let orientation =
            UnitQuaternion::try_from(OrientationConstraintQuaternion(msg.orientation))?;
        let tolerance = tolerance_from_wire(
            msg.parameterization,
            msg.absolute_x_axis_tolerance,
            msg.absolute_y_axis_tolerance,
            msg.absolute_z_axis_tolerance,
        )?;
        CoreOrientationConstraint::new(
            model,
            &tf,
            &msg.link_name,
            &msg.header.frame_id,
            orientation,
            tolerance,
            msg.weight,
        )
    }
}

impl TryFrom<CoreOrientationConstraint> for OrientationConstraintMsgOut {
    type Error = Error;

    /// Total: `desired_rotation_matrix_in_ref_frame()` is always a valid
    /// rotation matrix, and `UnitQuaternion::from_rotation_matrix` cannot
    /// fail on one.
    fn try_from(c: CoreOrientationConstraint) -> Result<Self, Self::Error> {
        let (parameterization, x, y, z) = tolerance_to_wire(c.tolerance());
        let orientation = Quaternion::try_from(UnitQuaternion::from_rotation_matrix(
            &c.desired_rotation_matrix_in_ref_frame(),
        ))?
        .0;
        Ok(OrientationConstraintMsgOut(
            moveit_msgs::OrientationConstraint {
                header: r2r::std_msgs::msg::Header {
                    frame_id: c.reference_frame().to_string(),
                    ..Default::default()
                },
                orientation,
                link_name: c.link_name().to_string(),
                absolute_x_axis_tolerance: x,
                absolute_y_axis_tolerance: y,
                absolute_z_axis_tolerance: z,
                parameterization,
                weight: c.weight(),
            },
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;

    fn valid_msg(model: &RobotModel) -> moveit_msgs::OrientationConstraint {
        moveit_msgs::OrientationConstraint {
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
            parameterization: XYZ_EULER_ANGLES,
            weight: 1.0,
        }
    }

    #[test]
    fn converts_with_xyz_euler_parameterization() {
        let model = one_joint_model();
        let c = CoreOrientationConstraint::try_from(OrientationConstraintMsg {
            model: &model,
            msg: valid_msg(&model),
        })
        .unwrap();
        assert!(matches!(
            c.tolerance(),
            OrientationTolerance::XyzEuler { .. }
        ));
    }

    #[test]
    fn invalid_parameterization_is_rejected() {
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.parameterization = 2;
        let err =
            CoreOrientationConstraint::try_from(OrientationConstraintMsg { model: &model, msg })
                .unwrap_err();
        // Not just the variant: `UnitQuaternion::try_from(OrientationConstraintQuaternion(..))`
        // runs before `tolerance_from_wire` in the same function and is a
        // sibling Error::Construct site (the two `degenerate_*`/`norm_2_*`
        // tests below), indistinguishable from this one by variant alone.
        assert!(
            err.to_string()
                .contains("is neither XYZ_EULER_ANGLES(0) nor ROTATION_VECTOR(1)"),
            "got: {err:?}"
        );
    }

    #[test]
    fn degenerate_orientation_is_rejected() {
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.orientation = r2r::geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        let err =
            CoreOrientationConstraint::try_from(OrientationConstraintMsg { model: &model, msg })
                .unwrap_err();
        // Not just the variant: `tolerance_from_wire` (above in this file) has
        // a sibling Error::Construct site, hit by
        // `invalid_parameterization_is_rejected`.
        assert!(
            err.to_string().contains("more than 1e-3 from 1.0"),
            "got: {err:?}"
        );
    }

    #[test]
    fn orientation_norm_2_is_rejected_end_to_end_unlike_a_scene_pose() {
        // §211: pins which rule this wire path actually uses. A scene Pose
        // at the same norm (`geometry.rs`'s
        // `pose_with_norm_2_orientation_succeeds_and_normalizes`) succeeds
        // and normalizes; this field must still reject it, since it goes
        // through `OrientationConstraintQuaternion`, not the generic rule.
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.orientation = r2r::geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 2.0,
        };
        let err =
            CoreOrientationConstraint::try_from(OrientationConstraintMsg { model: &model, msg })
                .unwrap_err();
        // Not just the variant: `tolerance_from_wire` (above in this file) has
        // a sibling Error::Construct site, hit by
        // `invalid_parameterization_is_rejected`.
        assert!(
            err.to_string().contains("more than 1e-3 from 1.0"),
            "got: {err:?}"
        );
    }

    #[test]
    fn unknown_frame_is_rejected() {
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.header.frame_id = "no_such_frame".to_string();
        let err =
            CoreOrientationConstraint::try_from(OrientationConstraintMsg { model: &model, msg })
                .unwrap_err();
        // Not just the variant: `OrientationConstraint::new` (moveit-constraints)
        // has a sibling `Error::UnknownName` site (`model.link_model(link_name)`,
        // kind "link") -- only the `kind` field tells this test apart from an
        // unknown `link_name` instead of an unknown `frame_id`.
        assert!(
            matches!(&err, Error::UnknownName { kind, .. } if *kind == "frame"),
            "got: {err:?}"
        );
    }

    #[test]
    fn round_trip_through_msg() {
        let model = one_joint_model();
        let c = CoreOrientationConstraint::try_from(OrientationConstraintMsg {
            model: &model,
            msg: valid_msg(&model),
        })
        .unwrap();
        let back = OrientationConstraintMsgOut::try_from(c).unwrap().0;
        assert_eq!(back.link_name, "tip");
        assert_eq!(back.parameterization, XYZ_EULER_ANGLES);
    }
}
