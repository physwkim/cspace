// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/PositionConstraint` <-> [`moveit_constraints::PositionConstraint`].
//! See `doc/message-mapping.md` §5.
//!
//! Scope this round: `shape_msgs/SolidPrimitive` only (BOX/SPHERE/CYLINDER/
//! CONE) -- `constraint_region.meshes[]` is rejected explicitly (`Error::Other`,
//! not silently dropped), matching this round's brief (`moveit-collision`'s
//! own mesh-shape mapping is out of the requested crate survey).
//!
//! **CONE is parseable but not usable end-to-end** (round 5, previously
//! undocumented): [`TryFrom<SolidPrimitiveMsg> for Shape`] below happily
//! builds a [`Shape::Cone`], but `moveit_constraints::PositionConstraint::new`
//! then calls `Body::from_shape`, which returns `Ok(None)` for
//! [`Shape::Cone`] (`moveit_geometry::bodies::Body` has no `Cone` variant),
//! so every `CONE`-typed constraint region fails there instead. This has
//! been true since round 2; see `doc/message-mapping.md` §5's `SolidPrimitive`
//! table for the full citation. Expires if `moveit_geometry::Body` grows a
//! `Cone` variant -- `moveit-geometry`'s call, not this crate's.

use moveit_error::Error;
use moveit_geometry::bodies::Body;
use moveit_geometry::{Cone, Cuboid, Cylinder, Isometry3, Shape, Sphere, Vector3 as CoreVector3};
use moveit_model::RobotModel;
use r2r::moveit_msgs::msg as moveit_msgs;
use r2r::shape_msgs::msg as shape_msgs;

use super::context::minimal_transforms;
use crate::geometry::{Pose, Vector3};

const BOX: u8 = 1;
const SPHERE: u8 = 2;
const CYLINDER: u8 = 3;
const CONE: u8 = 4;
const PRISM: u8 = 5;

const BOX_X: usize = 0;
const BOX_Y: usize = 1;
const BOX_Z: usize = 2;
const SPHERE_RADIUS: usize = 0;
const CYLINDER_HEIGHT: usize = 0;
const CYLINDER_RADIUS: usize = 1;
const CONE_HEIGHT: usize = 0;
const CONE_RADIUS: usize = 1;

/// Wraps `shape_msgs::msg::SolidPrimitive`.
pub struct SolidPrimitiveMsg(pub shape_msgs::SolidPrimitive);

fn dim(dims: &[f64], index: usize, field: &'static str) -> Result<f64, Error> {
    dims.get(index).copied().ok_or_else(|| {
        Error::construct(format!(
            "SolidPrimitive.dimensions has length {} but index {index} \
             ({field}) is required",
            dims.len()
        ))
    })
}

impl TryFrom<SolidPrimitiveMsg> for Shape {
    type Error = Error;

    /// `CYLINDER`/`CONE`'s wire dimension order is `[HEIGHT, RADIUS]`
    /// (`CYLINDER_HEIGHT=0, CYLINDER_RADIUS=1` per
    /// `shape_msgs/SolidPrimitive.msg`'s own constants) -- the reverse of
    /// `Cylinder::new(radius, length)`'s argument order. A naive
    /// `dimensions[0]` -> radius mapping would swap radius and length,
    /// same landmine shape as `SensorViewDirection`'s wire-vs-declared-order
    /// mismatch.
    fn try_from(msg: SolidPrimitiveMsg) -> Result<Self, Self::Error> {
        let d = &msg.0.dimensions;
        match msg.0.type_ {
            BOX => Ok(Shape::Cuboid(Cuboid::new(
                dim(d, BOX_X, "BOX_X")?,
                dim(d, BOX_Y, "BOX_Y")?,
                dim(d, BOX_Z, "BOX_Z")?,
            )?)),
            SPHERE => Ok(Shape::Sphere(Sphere::new(dim(
                d,
                SPHERE_RADIUS,
                "SPHERE_RADIUS",
            )?)?)),
            CYLINDER => Ok(Shape::Cylinder(Cylinder::new(
                dim(d, CYLINDER_RADIUS, "CYLINDER_RADIUS")?,
                dim(d, CYLINDER_HEIGHT, "CYLINDER_HEIGHT")?,
            )?)),
            CONE => Ok(Shape::Cone(Cone::new(
                dim(d, CONE_RADIUS, "CONE_RADIUS")?,
                dim(d, CONE_HEIGHT, "CONE_HEIGHT")?,
            )?)),
            PRISM => Err(Error::other(
                "SolidPrimitive.type=PRISM(5) has no moveit_geometry::Shape \
                 counterpart",
            )),
            other => Err(Error::construct(format!(
                "SolidPrimitive.type={other} is none of \
                 BOX(1)/SPHERE(2)/CYLINDER(3)/CONE(4)/PRISM(5)"
            ))),
        }
    }
}

fn body_to_solid_primitive(body: &Body) -> Result<shape_msgs::SolidPrimitive, Error> {
    let (type_, dimensions) = match body {
        Body::Sphere(s) => (SPHERE, s.dimensions()),
        Body::Cylinder(c) => {
            let d = c.dimensions(); // [radius, length]
            (CYLINDER, vec![d[1], d[0]]) // wire wants [height, radius]
        }
        Body::Cuboid(b) => (BOX, b.dimensions()), // [length, width, height] == [x, y, z]
        Body::ConvexMesh(_) => {
            return Err(Error::other(
                "Body::ConvexMesh has no SolidPrimitive representation; \
                 mesh round-tripping is not implemented this round",
            ));
        }
    };
    Ok(shape_msgs::SolidPrimitive {
        type_,
        dimensions,
        polygon: Default::default(),
    })
}

/// Wraps the wire message with the `&RobotModel` needed to resolve
/// `link_name`/`header.frame_id` (§5).
pub struct PositionConstraintMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: moveit_msgs::PositionConstraint,
}

/// Plain local wrapper, for the core->msg direction.
pub struct PositionConstraintMsgOut(pub moveit_msgs::PositionConstraint);

impl<'m> TryFrom<PositionConstraintMsg<'m>> for moveit_constraints::PositionConstraint {
    type Error = Error;

    fn try_from(wrapped: PositionConstraintMsg<'m>) -> Result<Self, Self::Error> {
        let PositionConstraintMsg { model, msg } = wrapped;
        let tf = minimal_transforms(model)?;
        let region = msg.constraint_region;

        if region.primitives.len() != region.primitive_poses.len() {
            return Err(Error::construct(format!(
                "BoundingVolume.primitives has length {} but \
                 primitive_poses has length {}",
                region.primitives.len(),
                region.primitive_poses.len()
            )));
        }
        if !region.meshes.is_empty() || !region.mesh_poses.is_empty() {
            return Err(Error::other(
                "BoundingVolume.meshes is not supported this round (see \
                 doc/message-mapping.md §5)",
            ));
        }

        let mut shapes = Vec::with_capacity(region.primitives.len());
        for (primitive, pose) in region.primitives.into_iter().zip(region.primitive_poses) {
            let shape = Shape::try_from(SolidPrimitiveMsg(primitive))?;
            let iso = Isometry3::try_from(Pose(pose))?;
            shapes.push((shape, iso));
        }

        let offset = CoreVector3::try_from(Vector3(msg.target_point_offset))?;
        moveit_constraints::PositionConstraint::new(
            model,
            &tf,
            &msg.link_name,
            &msg.header.frame_id,
            offset,
            &shapes,
            msg.weight,
        )
    }
}

impl TryFrom<moveit_constraints::PositionConstraint> for PositionConstraintMsgOut {
    type Error = Error;

    /// Fails only if a region's [`Body`] is a [`Body::ConvexMesh`] (never
    /// produced by this round's msg->core direction, since that direction
    /// only ever builds `Sphere`/`Cylinder`/`Cuboid` bodies from
    /// `SolidPrimitive`s -- but a `PositionConstraint` built directly by
    /// other core code, not from a message, could still carry one).
    fn try_from(c: moveit_constraints::PositionConstraint) -> Result<Self, Self::Error> {
        let mut primitives = Vec::with_capacity(c.constraint_regions().len());
        let mut primitive_poses = Vec::with_capacity(c.constraint_regions().len());
        for region in c.constraint_regions() {
            primitives.push(body_to_solid_primitive(&region.body)?);
            primitive_poses.push(Pose::try_from(region.pose)?.0);
        }
        let target_point_offset = if c.has_link_offset() {
            Vector3::try_from(c.link_offset())?.0
        } else {
            Default::default()
        };
        Ok(PositionConstraintMsgOut(moveit_msgs::PositionConstraint {
            header: r2r::std_msgs::msg::Header {
                frame_id: c.reference_frame().to_string(),
                ..Default::default()
            },
            link_name: c.link_name().to_string(),
            target_point_offset,
            constraint_region: moveit_msgs::BoundingVolume {
                primitives,
                primitive_poses,
                meshes: Vec::new(),
                mesh_poses: Vec::new(),
            },
            weight: c.weight(),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;

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

    fn sphere_primitive(radius: f64) -> shape_msgs::SolidPrimitive {
        shape_msgs::SolidPrimitive {
            type_: SPHERE,
            dimensions: vec![radius],
            polygon: Default::default(),
        }
    }

    #[test]
    fn solid_primitive_cylinder_dimension_order_is_height_then_radius() {
        // wire: [CYLINDER_HEIGHT=0, CYLINDER_RADIUS=1] = [2.0, 5.0] means
        // height=2.0, radius=5.0 -- not radius=2.0, height=5.0.
        let msg = shape_msgs::SolidPrimitive {
            type_: CYLINDER,
            dimensions: vec![2.0, 5.0],
            polygon: Default::default(),
        };
        let shape = Shape::try_from(SolidPrimitiveMsg(msg)).unwrap();
        match shape {
            Shape::Cylinder(c) => {
                assert_eq!(c.radius, 5.0);
                assert_eq!(c.length, 2.0);
            }
            other => panic!("expected Cylinder, got {other:?}"),
        }
    }

    #[test]
    fn solid_primitive_cone_dimension_order_is_height_then_radius() {
        // wire: [CONE_HEIGHT=0, CONE_RADIUS=1] = [2.0, 5.0] means
        // height=2.0, radius=5.0 -- not radius=2.0, height=5.0. Same
        // landmine as CYLINDER above; never had a test until round 5.
        let msg = shape_msgs::SolidPrimitive {
            type_: CONE,
            dimensions: vec![2.0, 5.0],
            polygon: Default::default(),
        };
        let shape = Shape::try_from(SolidPrimitiveMsg(msg)).unwrap();
        match shape {
            Shape::Cone(c) => {
                assert_eq!(c.radius, 5.0);
                assert_eq!(c.length, 2.0);
            }
            other => panic!("expected Cone, got {other:?}"),
        }
    }

    #[test]
    fn prism_is_rejected() {
        let msg = shape_msgs::SolidPrimitive {
            type_: PRISM,
            dimensions: vec![1.0],
            polygon: Default::default(),
        };
        let err = Shape::try_from(SolidPrimitiveMsg(msg)).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
    }

    #[test]
    fn short_dimensions_array_is_rejected_not_panicking() {
        let msg = shape_msgs::SolidPrimitive {
            type_: BOX,
            dimensions: vec![1.0],
            polygon: Default::default(),
        };
        let err = Shape::try_from(SolidPrimitiveMsg(msg)).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    fn valid_msg(model: &RobotModel) -> moveit_msgs::PositionConstraint {
        moveit_msgs::PositionConstraint {
            header: r2r::std_msgs::msg::Header {
                frame_id: model.model_frame().to_string(),
                ..Default::default()
            },
            link_name: "tip".to_string(),
            target_point_offset: r2r::geometry_msgs::msg::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            constraint_region: moveit_msgs::BoundingVolume {
                primitives: vec![sphere_primitive(0.05)],
                primitive_poses: vec![identity_pose()],
                meshes: vec![],
                mesh_poses: vec![],
            },
            weight: 1.0,
        }
    }

    #[test]
    fn converts_with_model_context() {
        let model = one_joint_model();
        let c = moveit_constraints::PositionConstraint::try_from(PositionConstraintMsg {
            model: &model,
            msg: valid_msg(&model),
        })
        .unwrap();
        assert_eq!(c.constraint_regions().len(), 1);
    }

    #[test]
    fn cone_constraint_region_is_rejected_end_to_end() {
        // Shape::try_from succeeds for CONE (see the dimension-order test
        // above), but moveit_geometry::Body has no Cone variant --
        // Body::from_shape returns Ok(None), which PositionConstraint::new
        // turns into an error. This is the previously-undocumented gap
        // named in this module's doc comment, not a regression.
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.constraint_region.primitives = vec![shape_msgs::SolidPrimitive {
            type_: CONE,
            dimensions: vec![1.0, 1.0],
            polygon: Default::default(),
        }];
        let err = moveit_constraints::PositionConstraint::try_from(PositionConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn mismatched_primitive_and_pose_lengths_is_rejected() {
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.constraint_region.primitive_poses.push(identity_pose());
        let err = moveit_constraints::PositionConstraint::try_from(PositionConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn meshes_are_rejected_not_silently_dropped() {
        let model = one_joint_model();
        let mut msg = valid_msg(&model);
        msg.constraint_region.meshes.push(Default::default());
        msg.constraint_region.mesh_poses.push(identity_pose());
        let err = moveit_constraints::PositionConstraint::try_from(PositionConstraintMsg {
            model: &model,
            msg,
        })
        .unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
    }

    #[test]
    fn round_trip_through_msg() {
        let model = one_joint_model();
        let c = moveit_constraints::PositionConstraint::try_from(PositionConstraintMsg {
            model: &model,
            msg: valid_msg(&model),
        })
        .unwrap();
        let back = PositionConstraintMsgOut::try_from(c).unwrap().0;
        assert_eq!(back.link_name, "tip");
        assert_eq!(back.constraint_region.primitives.len(), 1);
        assert_eq!(back.constraint_region.primitives[0].type_, SPHERE);
        assert_eq!(back.constraint_region.primitives[0].dimensions, vec![0.05]);
    }
}
