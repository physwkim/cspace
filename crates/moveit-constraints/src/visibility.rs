// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   (class VisibilityConstraint)
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   (VisibilityConstraint::configure, VisibilityConstraint::decide,
//    up to and not including the cone-vs-robot collision check)

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Transforms};
use moveit_model::RobotModel;
use moveit_state::Posed;

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

/// What [`VisibilityConstraint::decide_geometry`] could determine without
/// performing the cone-vs-robot collision check upstream's `decide()`
/// finishes with. See the crate's module docs for why that check is not yet
/// implemented.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VisibilityDecision {
    /// The view-angle or range-angle checks alone already decided the
    /// outcome (either violated, or no radius was configured so upstream's
    /// `decide()` never reaches the cone check either).
    Decided(ConstraintEvaluationResult),
    /// Upstream would build the visibility cone and collision-check it
    /// against the robot here. This port has no collision backend to do
    /// that with yet (see the crate's module docs) — callers must not treat
    /// this as "satisfied".
    NeedsConeCollisionCheck,
}

/// Constrains a target disc to remain visible (unimpeded by the robot's own
/// links) from a sensor, optionally also constraining the sensor's and
/// target's relative viewing/range angles.
///
/// Upstream `kinematic_constraints::VisibilityConstraint`. See the crate's
/// module docs for what is and is not ported: the view-angle and
/// range-angle checks are complete; the cone-vs-robot collision check is
/// not (no collision backend exists yet in this port).
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
    /// stopping short of the cone-vs-robot collision check — see this
    /// type's and the crate's module docs.
    pub fn decide_geometry(&self, state: &Posed) -> VisibilityDecision {
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
                return VisibilityDecision::Decided(ConstraintEvaluationResult::new(false, 0.0));
            }
            let ang = dp.acos();
            if max_view_angle < ang {
                return VisibilityDecision::Decided(ConstraintEvaluationResult::new(false, 0.0));
            }
        }

        if let Some(max_range_angle) = self.max_range_angle {
            let dir = (world_to_target.translation.vector - world_to_sensor.translation.vector)
                .normalize();
            let dp = sensor_view_axis.dot(&dir);
            if dp < 0.0 {
                return VisibilityDecision::Decided(ConstraintEvaluationResult::new(false, 0.0));
            }
            let ang = dp.acos();
            if max_range_angle < ang {
                return VisibilityDecision::Decided(ConstraintEvaluationResult::new(false, 0.0));
            }
        }

        match self.target_radius {
            Some(_) => VisibilityDecision::NeedsConeCollisionCheck,
            None => VisibilityDecision::Decided(ConstraintEvaluationResult::new(true, 0.0)),
        }
    }
}
