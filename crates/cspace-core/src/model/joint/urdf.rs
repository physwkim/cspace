// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/src/robot_model.cpp (jointBoundsFromURDF, constructJointModel)

use crate::error::{Error, Result};
use crate::geometry::Vector3;

use super::bounds::VariableBounds;
use super::model::JointModel;

/// Build one [`JointModel`] from one `<joint>` element.
///
/// Upstream `RobotModel::constructJointModel`'s `switch (parent_joint->type)`
/// arm, taken in isolation from the tree-walking it is embedded in. Upstream
/// receives `child_link` and reaches `parent_joint` through it, and falls
/// back to the SRDF's virtual joint list when `parent_joint` is null (the
/// root link); both of those require the full link graph, which is
/// `RobotModel`'s job, not this crate's — callers here already have the
/// `urdf_rs::Joint` in hand (e.g. from walking `urdf_rs::Robot::joints`
/// themselves, or constructing a virtual joint by hand from the SRDF).
///
/// `limit_present` must be whether the original `<joint>` element had a
/// `<limit>` child at all — see `joint_bounds_from_urdf`'s doc comment for
/// why this can't be recovered from `joint.limit` alone, and thus must come
/// from the caller.
///
/// # Errors
///
/// [`Error::construct`] if `joint.joint_type` is [`urdf_rs::JointType::Spherical`],
/// which has no MoveIt equivalent.
///
/// # Deviation from upstream
///
/// Verified against `urdfdom_headers/urdf_model/joint.h:173-175`: upstream's
/// own `urdf::Joint::Type` enum is `UNKNOWN, REVOLUTE, CONTINUOUS,
/// PRISMATIC, FLOATING, PLANAR, FIXED` — it has no `SPHERICAL` value at all,
/// so `constructJointModel`'s switch (`robot_model.cpp:942-980`) can never
/// receive one; its `default:` arm (reached only for `UNKNOWN`) logs
/// `RCLCPP_ERROR("Unknown joint type: %d")` and leaves `new_joint_model`
/// `nullptr` rather than raising any typed error. `Spherical` exists here
/// only because `urdf_rs` (this port's URDF parser, a different library from
/// upstream's `urdfdom`) has a variant upstream's enum lacks entirely — this
/// is not upstream declining to support a type it has, it is this port
/// rejecting a type upstream's own type system cannot express.
pub fn joint_model_from_urdf(joint: &urdf_rs::Joint, limit_present: bool) -> Result<JointModel> {
    use urdf_rs::JointType as UrdfJointType;

    let mut model = match joint.joint_type {
        UrdfJointType::Revolute => {
            let mut model = JointModel::new_revolute(joint.name.clone());
            model
                .set_variable_bounds(&joint.name, joint_bounds_from_urdf(joint, limit_present))
                .expect("just constructed with this exact variable name");
            model
                .set_continuous(false)
                .expect("just constructed as revolute");
            model
                .as_revolute_mut()
                .expect("just constructed as revolute")
                .set_axis(axis_vector(joint));
            model
        }
        UrdfJointType::Continuous => {
            let mut model = JointModel::new_revolute(joint.name.clone());
            model
                .set_variable_bounds(&joint.name, joint_bounds_from_urdf(joint, limit_present))
                .expect("just constructed with this exact variable name");
            model
                .set_continuous(true)
                .expect("just constructed as revolute");
            model
                .as_revolute_mut()
                .expect("just constructed as revolute")
                .set_axis(axis_vector(joint));
            model
        }
        UrdfJointType::Prismatic => {
            let mut model = JointModel::new_prismatic(joint.name.clone());
            model
                .set_variable_bounds(&joint.name, joint_bounds_from_urdf(joint, limit_present))
                .expect("just constructed with this exact variable name");
            model
                .as_prismatic_mut()
                .expect("just constructed as prismatic")
                .set_axis(axis_vector(joint));
            model
        }
        UrdfJointType::Fixed => JointModel::new_fixed(joint.name.clone()),
        UrdfJointType::Floating => JointModel::new_floating(joint.name.clone()),
        UrdfJointType::Planar => JointModel::new_planar(joint.name.clone()),
        UrdfJointType::Spherical => {
            return Err(Error::construct(format!(
                "joint '{}': spherical joints have no MoveIt equivalent",
                joint.name
            )));
        }
    };

    if let Some(mimic) = &joint.mimic {
        model.set_mimic(
            mimic.joint.clone(),
            mimic.multiplier.unwrap_or(1.0),
            mimic.offset.unwrap_or(0.0),
        );
    }

    Ok(model)
}

fn axis_vector(joint: &urdf_rs::Joint) -> Vector3 {
    let [x, y, z] = joint.axis.xyz.0;
    Vector3::new(x, y, z)
}

/// Upstream `jointBoundsFromURDF`: bounds for a 1-DOF joint (revolute,
/// continuous or prismatic).
///
/// `limit_present` stands in for upstream's null check on `urdf_joint->limits`:
/// `urdf_rs::Joint::limit` is `#[serde(default)]` rather than `Option`, so a
/// missing `<limit>` element and an explicit all-zero one both deserialize to
/// the same `JointLimit { lower: 0.0, upper: 0.0, .. }` — the caller
/// ([`RobotModel`](crate::model::RobotModel), via [`joint_model_from_urdf`]) must
/// recover the distinction from the raw URDF XML and pass it in, since
/// `joint.limit` alone cannot. When `limit_present` is `false`, every bound this function
/// would otherwise set from `joint.limit` is left at
/// [`VariableBounds::default`]'s unbounded value instead, matching upstream
/// leaving `position_bounded_`/`velocity_bounded_` at their default `false`
/// when `urdf_joint->limits` is null.
fn joint_bounds_from_urdf(joint: &urdf_rs::Joint, limit_present: bool) -> VariableBounds {
    let mut bounds = VariableBounds::default();
    if let Some(safety) = &joint.safety_controller {
        bounds.position_bounded = true;
        bounds.min_position = safety.soft_lower_limit;
        bounds.max_position = safety.soft_upper_limit;
        if limit_present {
            if joint.limit.lower > bounds.min_position {
                bounds.min_position = joint.limit.lower;
            }
            if joint.limit.upper < bounds.max_position {
                bounds.max_position = joint.limit.upper;
            }
        }
    } else if limit_present {
        bounds.position_bounded = true;
        bounds.min_position = joint.limit.lower;
        bounds.max_position = joint.limit.upper;
    }
    if limit_present {
        bounds.max_velocity = joint.limit.velocity.abs();
        bounds.min_velocity = -bounds.max_velocity;
        bounds.velocity_bounded = bounds.max_velocity > f64::EPSILON;
    }
    bounds
}

#[cfg(test)]
mod tests {
    use urdf_rs::{
        Joint, JointLimit, JointType as UrdfJointType, LinkName, Mimic as UrdfMimic, Pose,
        SafetyController,
    };

    use super::super::model::JointKind;
    use super::*;

    fn base_joint(joint_type: UrdfJointType) -> Joint {
        Joint {
            name: "j".to_string(),
            joint_type,
            origin: Pose::default(),
            parent: LinkName {
                link: "parent".to_string(),
            },
            child: LinkName {
                link: "child".to_string(),
            },
            axis: urdf_rs::Axis::default(),
            limit: JointLimit::default(),
            calibration: None,
            dynamics: None,
            mimic: None,
            safety_controller: None,
        }
    }

    #[test]
    fn bounds_come_from_limit_when_no_safety_controller() {
        let mut joint = base_joint(UrdfJointType::Revolute);
        joint.limit = JointLimit {
            lower: -2.0,
            upper: 2.0,
            velocity: 1.5,
            effort: 0.0,
        };
        let bounds = joint_bounds_from_urdf(&joint, true);
        assert!(bounds.position_bounded);
        assert_eq!(bounds.min_position, -2.0);
        assert_eq!(bounds.max_position, 2.0);
    }

    /// The boundary the panda/fanuc fixtures never exercise: a joint with no
    /// `<limit>` element at all must come out fully unbounded (matching
    /// upstream's null `urdf_joint->limits`), not clamped to the `[0, 0]`
    /// that `JointLimit::default()` happens to hold. `limit_present` is what
    /// distinguishes the two — `joint.limit` is identical in both cases.
    #[test]
    fn absent_limit_is_unbounded_not_a_zero_width_bound() {
        let mut joint = base_joint(UrdfJointType::Revolute);
        joint.limit = JointLimit::default();

        let absent = joint_bounds_from_urdf(&joint, false);
        assert!(!absent.position_bounded);
        assert!(!absent.velocity_bounded);

        let explicit_zero = joint_bounds_from_urdf(&joint, true);
        assert!(explicit_zero.position_bounded);
        assert_eq!(explicit_zero.min_position, 0.0);
        assert_eq!(explicit_zero.max_position, 0.0);
        assert!(!explicit_zero.velocity_bounded);

        assert_ne!(absent, explicit_zero);
    }

    #[test]
    fn bounds_use_soft_limit_when_narrower_than_hard_limit() {
        let mut joint = base_joint(UrdfJointType::Revolute);
        joint.limit = JointLimit {
            lower: -3.0,
            upper: 3.0,
            velocity: 1.0,
            effort: 0.0,
        };
        joint.safety_controller = Some(SafetyController {
            soft_lower_limit: -2.0,
            soft_upper_limit: 2.0,
            k_position: 0.0,
            k_velocity: 0.0,
        });
        let bounds = joint_bounds_from_urdf(&joint, true);
        assert_eq!(bounds.min_position, -2.0);
        assert_eq!(bounds.max_position, 2.0);
    }

    #[test]
    fn bounds_clamp_soft_limit_to_hard_limit_when_hard_limit_is_narrower() {
        let mut joint = base_joint(UrdfJointType::Revolute);
        joint.limit = JointLimit {
            lower: -1.0,
            upper: 1.0,
            velocity: 1.0,
            effort: 0.0,
        };
        joint.safety_controller = Some(SafetyController {
            soft_lower_limit: -5.0,
            soft_upper_limit: 5.0,
            k_position: 0.0,
            k_velocity: 0.0,
        });
        let bounds = joint_bounds_from_urdf(&joint, true);
        assert_eq!(bounds.min_position, -1.0);
        assert_eq!(bounds.max_position, 1.0);
    }

    #[test]
    fn velocity_bounded_is_false_at_zero_and_true_above_epsilon() {
        let mut joint = base_joint(UrdfJointType::Revolute);
        joint.limit.velocity = 0.0;
        assert!(!joint_bounds_from_urdf(&joint, true).velocity_bounded);

        joint.limit.velocity = 1.0;
        assert!(joint_bounds_from_urdf(&joint, true).velocity_bounded);
    }

    #[test]
    fn mimic_defaults_multiplier_and_offset_when_absent() {
        let mut joint = base_joint(UrdfJointType::Prismatic);
        joint.limit = JointLimit {
            lower: 0.0,
            upper: 1.0,
            velocity: 1.0,
            effort: 0.0,
        };
        joint.mimic = Some(UrdfMimic {
            joint: "other".to_string(),
            multiplier: None,
            offset: None,
        });
        let model = joint_model_from_urdf(&joint, true).unwrap();
        let mimic = model.mimic().expect("mimic was set");
        assert_eq!(mimic.joint_name, "other");
        assert_eq!(mimic.factor, 1.0);
        assert_eq!(mimic.offset, 0.0);
    }

    #[test]
    fn mimic_keeps_explicit_multiplier_and_offset() {
        let mut joint = base_joint(UrdfJointType::Prismatic);
        joint.limit = JointLimit {
            lower: 0.0,
            upper: 1.0,
            velocity: 1.0,
            effort: 0.0,
        };
        joint.mimic = Some(UrdfMimic {
            joint: "other".to_string(),
            multiplier: Some(2.0),
            offset: Some(0.1),
        });
        let model = joint_model_from_urdf(&joint, true).unwrap();
        let mimic = model.mimic().expect("mimic was set");
        assert_eq!(mimic.factor, 2.0);
        assert_eq!(mimic.offset, 0.1);
    }

    #[test]
    fn revolute_joint_type_is_not_continuous() {
        let mut joint = base_joint(UrdfJointType::Revolute);
        joint.limit = JointLimit {
            lower: -1.0,
            upper: 1.0,
            velocity: 1.0,
            effort: 0.0,
        };
        let model = joint_model_from_urdf(&joint, true).unwrap();
        assert!(!model.as_revolute().unwrap().is_continuous());
        assert!(model.variable_bounds()[0].position_bounded);
    }

    #[test]
    fn continuous_joint_type_is_continuous_and_unbounded() {
        let mut joint = base_joint(UrdfJointType::Continuous);
        joint.limit = JointLimit {
            lower: 0.0,
            upper: 0.0,
            velocity: 1.0,
            effort: 0.0,
        };
        let model = joint_model_from_urdf(&joint, true).unwrap();
        assert!(model.as_revolute().unwrap().is_continuous());
        assert!(!model.variable_bounds()[0].position_bounded);
    }

    #[test]
    fn prismatic_joint_axis_is_carried_through() {
        let mut joint = base_joint(UrdfJointType::Prismatic);
        joint.limit = JointLimit {
            lower: 0.0,
            upper: 0.04,
            velocity: 0.2,
            effort: 0.0,
        };
        joint.axis = urdf_rs::Axis {
            xyz: urdf_rs::Vec3([0.0, 0.0, 1.0]),
        };
        let model = joint_model_from_urdf(&joint, true).unwrap();
        let axis = model.as_prismatic().unwrap().axis();
        assert_eq!((axis.x, axis.y, axis.z), (0.0, 0.0, 1.0));
    }

    #[test]
    fn fixed_floating_and_planar_produce_the_matching_kind() {
        assert!(matches!(
            joint_model_from_urdf(&base_joint(UrdfJointType::Fixed), true)
                .unwrap()
                .kind(),
            JointKind::Fixed
        ));
        assert!(matches!(
            joint_model_from_urdf(&base_joint(UrdfJointType::Floating), true)
                .unwrap()
                .kind(),
            JointKind::Floating(_)
        ));
        assert!(matches!(
            joint_model_from_urdf(&base_joint(UrdfJointType::Planar), true)
                .unwrap()
                .kind(),
            JointKind::Planar(_)
        ));
    }

    #[test]
    fn spherical_joint_type_is_rejected() {
        assert!(joint_model_from_urdf(&base_joint(UrdfJointType::Spherical), true).is_err());
    }
}
