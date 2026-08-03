// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   (class VisibilityConstraint)
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   (VisibilityConstraint::configure, VisibilityConstraint::getVisibilityCone,
//    VisibilityConstraint::decide, VisibilityConstraint::decideContact)

use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, BodyType, CollisionEnv, CollisionRequest, Contact, DecideContactFn,
    LinkPaddingScale, ParryCollisionEnv, World,
};
use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Mesh, Shape, Transforms, Vector3};
use moveit_model::RobotModel;
use moveit_state::Posed;
use nalgebra::Point3;

use crate::ConstraintEvaluationResult;

const EPS: f64 = f64::EPSILON;

/// Which axis of the sensor pose's own frame points out of the sensor.
///
/// # Deviation from upstream: an enum, not a raw `int32`
///
/// `moveit_msgs::msg::VisibilityConstraint::sensor_view_direction` is an
/// `int32` matched against three `SENSOR_X`/`SENSOR_Y`/`SENSOR_Z` message
/// constants (`2`/`1`/`0`); nothing stops a caller passing `7`. This port
/// makes the three named directions the only representable values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorViewDirection {
    /// `moveit_msgs::msg::VisibilityConstraint::SENSOR_X`
    SensorX,
    /// `moveit_msgs::msg::VisibilityConstraint::SENSOR_Y`
    SensorY,
    /// `moveit_msgs::msg::VisibilityConstraint::SENSOR_Z`
    SensorZ,
}

impl SensorViewDirection {
    /// The column of the sensor's rotation matrix that points along this
    /// direction. Upstream indexes this as `col(2 - sensor_view_direction_)`
    /// (`SENSOR_Z = 0`, `SENSOR_Y = 1`, `SENSOR_X = 2`, so the subtraction
    /// picks column 2/1/0 respectively); this port names the column
    /// directly instead of keeping the upstream integer encoding and its
    /// subtraction around to reproduce.
    fn axis_column(self) -> usize {
        match self {
            Self::SensorX => 0,
            Self::SensorY => 1,
            Self::SensorZ => 2,
        }
    }
}

/// A pose given relative to a named frame, either already resolved into a
/// fixed frame or still to be resolved fresh from a state.
///
/// Same shape and same reason as `position::ReferenceFrame`/
/// `orientation::OrientationTarget` — see either's doc comment.
#[derive(Debug, Clone, PartialEq)]
enum FramedPose {
    /// `pose` is already expressed in `frame`, resolved once at
    /// construction.
    Fixed { frame: String, pose: Isometry3 },
    /// `pose` is relative to `frame` and must be composed with a fresh
    /// [`Posed::frame_transform`] lookup on every `decide_geometry` call.
    Mobile { frame: String, pose: Isometry3 },
}

impl FramedPose {
    fn new(model: &RobotModel, tf: &Transforms, frame_id: &str, pose: Isometry3) -> Result<Self> {
        if tf.can_transform(frame_id) {
            Ok(Self::Fixed {
                frame: tf.target_frame().to_string(),
                pose: tf.transform_pose(frame_id, &pose)?,
            })
        } else {
            if !model.has_link_model(frame_id) && frame_id != model.model_frame() {
                return Err(Error::unknown_name("frame", frame_id));
            }
            Ok(Self::Mobile {
                frame: frame_id.to_string(),
                pose,
            })
        }
    }

    fn frame(&self) -> &str {
        match self {
            Self::Fixed { frame, .. } | Self::Mobile { frame, .. } => frame,
        }
    }

    fn resolve(&self, state: &Posed) -> Isometry3 {
        match self {
            Self::Fixed { pose, .. } => *pose,
            Self::Mobile { frame, pose } => {
                state
                    .frame_transform(frame)
                    .expect("mobile reference frame was validated resolvable at construction")
                    * pose
            }
        }
    }
}

/// Constrains a target disc to remain visible (unimpeded by the robot's own
/// links) from a sensor, optionally also constraining the sensor's and
/// target's relative viewing/range angles.
///
/// Upstream `kinematic_constraints::VisibilityConstraint`, ported in full:
/// the view-angle and range-angle checks, and the cone-vs-robot collision
/// check ([`VisibilityConstraint::decide`] builds the visibility cone as a
/// [`Mesh`] and checks it against the robot via
/// `moveit_collision::ParryCollisionEnv`, mirroring upstream's local,
/// scoped `CollisionEnvFCL` — see that method's doc for why no
/// `PlanningScene`/broader collision world is needed here).
///
/// # Deviation from upstream: `Option<f64>` for the two angle limits, and
/// for the target radius
///
/// `moveit_msgs::msg::VisibilityConstraint::max_view_angle`/
/// `max_range_angle` are `0.0` to mean "this criterion is not checked" —
/// the same magic-zero-as-absence pattern the crate's other constraint
/// types replace with `Option`. `target_radius` has the identical shape
/// (`configure()`'s own `enabled()` treats `target_radius_ > eps` exactly
/// like the other two) even though the task that scoped this crate named
/// only the angle pair — the same defect family, so the same fix applies
/// here rather than leaving one of the three sentinel-zero fields
/// unrepaired. [`VisibilityConstraint::new`] takes all three as `Option<f64>`
/// and normalizes a `Some` value at or below `f64::EPSILON` to `None`, so
/// `Some` always means "this criterion is active" with no runtime
/// re-checking required at read time.
#[derive(Debug, Clone, PartialEq)]
pub struct VisibilityConstraint {
    sensor: FramedPose,
    sensor_view_direction: SensorViewDirection,
    target: FramedPose,
    cone_sides: usize,
    target_radius: Option<f64>,
    max_view_angle: Option<f64>,
    max_range_angle: Option<f64>,
    weight: f64,
}

fn normalize_criterion(value: Option<f64>) -> Option<f64> {
    value.map(f64::abs).filter(|v| *v > EPS)
}

/// Where the sensor is, and which of its own axes points along its view
/// direction. Grouped out of [`VisibilityConstraint::new`]'s argument list
/// because the two are read together everywhere: neither means anything
/// without the other.
#[derive(Debug, Clone, Copy)]
pub struct SensorSpec<'a> {
    /// Frame the sensor pose is relative to.
    pub frame_id: &'a str,
    /// Sensor pose in `frame_id`.
    pub pose: Isometry3,
    /// Which sensor-frame axis points out of the sensor.
    pub view_direction: SensorViewDirection,
}

/// Where the visibility target is. Grouped out of
/// [`VisibilityConstraint::new`]'s argument list for the same reason as
/// [`SensorSpec`].
#[derive(Debug, Clone, Copy)]
pub struct TargetSpec<'a> {
    /// Frame the target pose is relative to.
    pub frame_id: &'a str,
    /// Target pose in `frame_id`.
    pub pose: Isometry3,
}

/// The three optional visibility criteria upstream encodes as
/// `target_radius`/`max_view_angle`/`max_range_angle`, `0.0` meaning
/// "unconstrained" for all three (see this type's own doc comment on why
/// they are `Option<f64>` here). Grouped into one argument since they are
/// normalized identically and always supplied together.
#[derive(Debug, Clone, Copy, Default)]
pub struct VisibilityCriteria {
    /// Radius of the visibility target disc; `None` if unconstrained.
    pub target_radius: Option<f64>,
    /// Maximum angle between the sensor's view axis and the target's
    /// surface normal; `None` if unconstrained.
    pub max_view_angle: Option<f64>,
    /// Maximum angle between the sensor's view axis and the
    /// sensor-to-target direction; `None` if unconstrained.
    pub max_range_angle: Option<f64>,
}

impl VisibilityConstraint {
    /// Build and resolve a visibility constraint against `model`.
    ///
    /// `cone_sides` below 3 is raised to 3, matching upstream's own
    /// `configure()` (a real geometric floor, not a sentinel to repair).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `sensor.frame_id`/`target.frame_id` are
    /// empty and not the model frame or a link name.
    /// [`Error::Construct`] if `weight` is not strictly positive.
    pub fn new(
        model: &RobotModel,
        tf: &Transforms,
        sensor: SensorSpec,
        target: TargetSpec,
        cone_sides: usize,
        criteria: VisibilityCriteria,
        weight: f64,
    ) -> Result<Self> {
        if weight <= EPS {
            return Err(Error::construct(
                "VisibilityConstraint weight must be strictly positive",
            ));
        }
        Ok(Self {
            sensor: FramedPose::new(model, tf, sensor.frame_id, sensor.pose)?,
            sensor_view_direction: sensor.view_direction,
            target: FramedPose::new(model, tf, target.frame_id, target.pose)?,
            cone_sides: cone_sides.max(3),
            target_radius: normalize_criterion(criteria.target_radius),
            max_view_angle: normalize_criterion(criteria.max_view_angle),
            max_range_angle: normalize_criterion(criteria.max_range_angle),
            weight,
        })
    }

    /// `getSensorFrame`/`getTargetFrame` are each `mobileReferenceFrame`
    /// away from upstream's split fields — exposed together since both
    /// constraint targets share the same `FramedPose` shape here.
    pub fn sensor_frame(&self) -> &str {
        self.sensor.frame()
    }

    /// See [`VisibilityConstraint::sensor_frame`].
    pub fn target_frame(&self) -> &str {
        self.target.frame()
    }

    /// Number of sides used to approximate the visibility cone (always
    /// `>= 3`).
    pub fn cone_sides(&self) -> usize {
        self.cone_sides
    }

    /// `enabled`: whether any of the three criteria is active.
    pub fn enabled(&self) -> bool {
        self.target_radius.is_some()
            || self.max_view_angle.is_some()
            || self.max_range_angle.is_some()
    }

    /// The view-angle and range-angle checks from upstream's `decide()`,
    /// stopping short of the cone-vs-robot collision check. `Some` if these
    /// two checks alone already decide the outcome (either violated, or no
    /// radius was configured so upstream's `decide()` never reaches the
    /// cone check either); `None` means [`VisibilityConstraint::decide`]
    /// must go on to build and collision-check the cone.
    fn decide_by_angle(&self, state: &Posed) -> Option<ConstraintEvaluationResult> {
        let world_to_sensor = self.sensor.resolve(state);
        let world_to_target = self.target.resolve(state);

        let sensor_view_axis = world_to_sensor
            .rotation
            .to_rotation_matrix()
            .matrix()
            .column(self.sensor_view_direction.axis_column())
            .into_owned();

        if let Some(max_view_angle) = self.max_view_angle {
            let target_z = world_to_target
                .rotation
                .to_rotation_matrix()
                .matrix()
                .column(2)
                .into_owned();
            let normal1 = -target_z;
            let dp = sensor_view_axis.dot(&normal1);
            if dp < 0.0 {
                return Some(ConstraintEvaluationResult::new(false, 0.0));
            }
            let ang = dp.acos();
            if max_view_angle < ang {
                return Some(ConstraintEvaluationResult::new(false, 0.0));
            }
        }

        if let Some(max_range_angle) = self.max_range_angle {
            let dir = (world_to_target.translation.vector - world_to_sensor.translation.vector)
                .normalize();
            let dp = sensor_view_axis.dot(&dir);
            if dp < 0.0 {
                return Some(ConstraintEvaluationResult::new(false, 0.0));
            }
            let ang = dp.acos();
            if max_range_angle < ang {
                return Some(ConstraintEvaluationResult::new(false, 0.0));
            }
        }

        match self.target_radius {
            Some(_) => None,
            None => Some(ConstraintEvaluationResult::new(true, 0.0)),
        }
    }

    /// `getVisibilityCone(tform_world_to_sensor, tform_world_to_target)`: the
    /// mesh cone upstream collision-checks against the robot — apex at the
    /// sensor origin, base a `cone_sides`-gon of radius `target_radius`
    /// centered on the target, plus one extra vertex at the target center
    /// itself (upstream's `points_[1]`, used by the base triangles).
    /// Vertex/triangle indices below follow upstream's exactly: `0` sensor,
    /// `1` target center, `2..cone_sides+2` the disc rim, closing the loop
    /// between the last and first rim points with the two triangles
    /// upstream computes outside its main loop.
    fn cone_mesh(&self, world_to_sensor: Isometry3, world_to_target: Isometry3) -> Mesh {
        let target_radius = self
            .target_radius
            .expect("only called from decide() after decide_by_angle found a radius configured");

        let mut vertices = Vec::with_capacity(self.cone_sides + 2);
        vertices.push(world_to_sensor.translation.vector);
        vertices.push(world_to_target.translation.vector);
        let delta = 2.0 * std::f64::consts::PI / self.cone_sides as f64;
        for i in 0..self.cone_sides {
            let a = delta * i as f64;
            let rim_point_in_target =
                Vector3::new(a.sin() * target_radius, a.cos() * target_radius, 0.0);
            vertices.push((world_to_target * Point3::from(rim_point_in_target)).coords);
        }

        let mut triangles = Vec::with_capacity(self.cone_sides * 2);
        for i in 1..self.cone_sides {
            triangles.push([(i + 1) as u32, 0, (i + 2) as u32]);
            triangles.push([(i + 1) as u32, 1, (i + 2) as u32]);
        }
        triangles.push([(self.cone_sides + 1) as u32, 0, 2]);
        triangles.push([(self.cone_sides + 1) as u32, 1, 2]);

        Mesh::new(vertices, triangles)
            .expect("every triangle index above is < cone_sides + 2, the vertex count just built")
    }

    /// `decide(state, verbose)`, ported in full. The view-angle/range-angle
    /// checks (`decide_by_angle`) run first, matching upstream's early
    /// returns; only when a `target_radius` is configured and those two
    /// checks pass does this go on to build the cone (`cone_mesh`) and
    /// collision-check it.
    ///
    /// # Why no `PlanningScene`/broader collision world is needed
    ///
    /// Upstream's cone check does not use the caller's own collision
    /// world — it builds a brand new, throwaway
    /// `collision_detection::CollisionEnvFCL(robot_model_)`, adds the cone
    /// as that local environment's only world object, checks the robot
    /// against it, then discards the whole thing. This method reproduces
    /// exactly that: a fresh `moveit_collision::World` holding only the
    /// cone, a fresh `ParryCollisionEnv` over it (default, untracked
    /// [`LinkPaddingScale`] — untracked already reports the same
    /// padding-`0.0`/scale-`1.0` upstream's default-constructed
    /// `CollisionEnvFCL` uses), and a fresh
    /// [`AllowedCollisionMatrix`] with one default conditional entry for
    /// `"cone"`. None of this depends on `moveit-scene`'s `PlanningScene`
    /// (not yet built by this port) or any collision state the caller
    /// might be tracking elsewhere.
    pub fn decide(&self, state: &Posed) -> ConstraintEvaluationResult {
        let Some(result) = self.decide_by_angle(state) else {
            return self.decide_cone(state);
        };
        result
    }

    fn decide_cone(&self, state: &Posed) -> ConstraintEvaluationResult {
        let world_to_sensor = self.sensor.resolve(state);
        let world_to_target = self.target.resolve(state);
        let cone = self.cone_mesh(world_to_sensor, world_to_target);

        let mut world = World::new();
        world.add_shape("cone", Arc::new(Shape::Mesh(cone)), Isometry3::identity());
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::new());

        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_conditional_entry(
            "cone",
            allow_sensor_or_target_contact(
                self.sensor_frame().to_owned(),
                self.target_frame().to_owned(),
            ),
        );

        let request = CollisionRequest {
            contacts: true,
            max_contacts: 1,
            ..Default::default()
        };
        let result = env.check_robot_collision(&request, state, &[], Some(&acm));

        let depth = result
            .contacts
            .as_ref()
            .and_then(|contacts| contacts.by_pair.values().next())
            .and_then(|pair| pair.first())
            .map_or(0.0, |contact| contact.depth);
        ConstraintEvaluationResult::new(
            !result.collision,
            if result.collision { depth } else { 0.0 },
        )
    }
}

/// `decideContact`: a contact with the cone is ignored (does not make the
/// constraint violated) when either body is a robot-attached object
/// (upstream allows these unconditionally, regardless of name), or when the
/// robot-link side of the contact is named the same as the sensor or
/// target frame (the sensor/target links themselves necessarily touch the
/// cone at its apex/base-center vertices, which is not the occlusion this
/// constraint checks for).
fn allow_sensor_or_target_contact(sensor_frame: String, target_frame: String) -> DecideContactFn {
    Arc::new(move |contact: &mut Contact| {
        if contact.body_type_1 == BodyType::RobotAttached
            || contact.body_type_2 == BodyType::RobotAttached
        {
            return true;
        }
        if contact.body_type_1 == BodyType::RobotLink
            && contact.body_type_2 == BodyType::WorldObject
            && (Transforms::same_frame(&contact.body_name_1, &sensor_frame)
                || Transforms::same_frame(&contact.body_name_1, &target_frame))
        {
            return true;
        }
        if contact.body_type_2 == BodyType::RobotLink
            && contact.body_type_1 == BodyType::WorldObject
            && (Transforms::same_frame(&contact.body_name_2, &sensor_frame)
                || Transforms::same_frame(&contact.body_name_2, &target_frame))
        {
            return true;
        }
        false
    })
}
