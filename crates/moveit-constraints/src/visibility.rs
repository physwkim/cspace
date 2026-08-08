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

    /// The pose as stored: for `Fixed`, already transformed into
    /// [`FramedPose::frame`] at construction; for `Mobile`, as given
    /// relative to `frame`, untransformed. Same split as
    /// `orientation::OrientationTarget::{Fixed,Mobile}` — see that type's
    /// `desired_rotation_matrix` doc comment for why neither branch needs a
    /// live [`Posed`] to answer this.
    fn pose(&self) -> Isometry3 {
        match self {
            Self::Fixed { pose, .. } | Self::Mobile { pose, .. } => *pose,
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
/// and normalizes a `Some` at or below `f64::EPSILON` to `None`, so `Some`
/// always means "this criterion is active" with no runtime re-checking
/// required at read time.
///
/// The three are not normalized identically, matching an asymmetry already
/// present upstream: `target_radius_` is assigned `fabs(vc.target_radius)`
/// (`kinematic_constraint.cpp:818`), so a negative wire value still
/// activates the criterion at its magnitude; `max_view_angle_`/
/// `max_range_angle_` are assigned straight from the message with no
/// `fabs()` (`:879-880`), so a negative wire value fails the `> eps` gate
/// and leaves the criterion inactive. See `normalize_target_radius`/
/// `normalize_angle_criterion` (private, this module) for exactly this
/// split.
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

/// `target_radius_ = fabs(vc.target_radius)` (`kinematic_constraint.cpp:818`)
/// is always non-negative going in, then gated by `enabled()`/`decide()`'s
/// `target_radius_ > eps` check — so a negative wire value still activates
/// the criterion, at its magnitude.
fn normalize_target_radius(value: Option<f64>) -> Option<f64> {
    value.map(f64::abs).filter(|v| *v > EPS)
}

/// `max_view_angle_`/`max_range_angle_` are assigned straight from the
/// message with no `fabs()` (`:879-880`, unlike `target_radius_` above),
/// then gated by the same `> eps` check (`:1081`, `:1109`) -- so a negative
/// wire value fails that gate and the criterion is inactive, not active at
/// its magnitude. Deliberately not the same function as
/// [`normalize_target_radius`]: applying `.abs()` here would flip a
/// upstream-inactive negative value into an active positive threshold.
fn normalize_angle_criterion(value: Option<f64>) -> Option<f64> {
    value.filter(|v| *v > EPS)
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
    /// `weight <= f64::EPSILON` (including negative weights) normalizes to
    /// `1.0` rather than erroring — see
    /// [`crate::JointConstraint::new`]'s "Weight normalization" doc section
    /// for the full rationale and the D6/D14 boundary this is not D6.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `sensor.frame_id`/`target.frame_id` are
    /// empty and not the model frame or a link name.
    pub fn new(
        model: &RobotModel,
        tf: &Transforms,
        sensor: SensorSpec,
        target: TargetSpec,
        cone_sides: usize,
        criteria: VisibilityCriteria,
        weight: f64,
    ) -> Result<Self> {
        let weight = if weight <= EPS { 1.0 } else { weight };
        Ok(Self {
            sensor: FramedPose::new(model, tf, sensor.frame_id, sensor.pose)?,
            sensor_view_direction: sensor.view_direction,
            target: FramedPose::new(model, tf, target.frame_id, target.pose)?,
            cone_sides: cone_sides.max(3),
            target_radius: normalize_target_radius(criteria.target_radius),
            max_view_angle: normalize_angle_criterion(criteria.max_view_angle),
            max_range_angle: normalize_angle_criterion(criteria.max_range_angle),
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

    /// Not an upstream accessor: every field below `enabled()` on this type
    /// is `protected` upstream with no getter at all (`sensor_pose_` etc.,
    /// `kinematic_constraint.hpp:870-882`), the same "field has no upstream
    /// getter" gap [`crate::JointConstraint::weight`]/
    /// [`crate::PositionConstraint::weight`]/
    /// [`crate::OrientationConstraint::weight`] already close for `weight_`
    /// — added here for the same reason: a caller building a
    /// `moveit_msgs`-shaped conversion needs every field back out, not just
    /// the ones `decide()` happens to read directly.
    ///
    /// The sensor pose, in [`VisibilityConstraint::sensor_frame`]. See
    /// `FramedPose::pose` for exactly what "in `sensor_frame`" means for
    /// the mobile-frame case.
    pub fn sensor(&self) -> Isometry3 {
        self.sensor.pose()
    }

    /// The target pose, in [`VisibilityConstraint::target_frame`]. See
    /// `FramedPose::pose` for exactly what "in `target_frame`" means for
    /// the mobile-frame case.
    pub fn target(&self) -> Isometry3 {
        self.target.pose()
    }

    /// `sensor_view_direction_`: which axis of [`VisibilityConstraint::sensor`]'s
    /// own frame points out of the sensor. Returned as the core
    /// [`SensorViewDirection`] enum, never as an integer — see that type's
    /// own doc comment for why upstream's `SENSOR_X`/`Y`/`Z` wire encoding
    /// (`2`/`1`/`0`) is the reverse of this enum's declaration order, which
    /// an integer return would invite reproducing by position.
    pub fn sensor_view_direction(&self) -> SensorViewDirection {
        self.sensor_view_direction
    }

    /// Radius of the visibility target disc; `None` if unconstrained. See
    /// this type's doc comment for why this is `Option<f64>` rather than
    /// upstream's magic-zero `target_radius_`.
    pub fn target_radius(&self) -> Option<f64> {
        self.target_radius
    }

    /// Maximum angle between the sensor's view axis and the target's
    /// surface normal; `None` if unconstrained. See this type's doc comment
    /// for why this is `Option<f64>` rather than upstream's magic-zero
    /// `max_view_angle_`.
    pub fn max_view_angle(&self) -> Option<f64> {
        self.max_view_angle
    }

    /// Maximum angle between the sensor's view axis and the
    /// sensor-to-target direction; `None` if unconstrained. See this type's
    /// doc comment for why this is `Option<f64>` rather than upstream's
    /// magic-zero `max_range_angle_`.
    pub fn max_range_angle(&self) -> Option<f64> {
        self.max_range_angle
    }

    /// Not an upstream accessor (`weight_` has none there either, same as
    /// [`crate::JointConstraint::weight`]/[`crate::PositionConstraint::weight`]/
    /// [`crate::OrientationConstraint::weight`]): exposed for the same
    /// reason those are — a caller building a `moveit_msgs`-shaped
    /// conversion needs every field back out, not just the ones `decide()`
    /// happens to read directly.
    pub fn weight(&self) -> f64 {
        self.weight
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
            // Eigen's `.normalized()` returns the input unchanged when
            // `squaredNorm()` is zero instead of dividing; nalgebra's
            // `.normalize()` is `unscale(self.norm())` with no such guard,
            // so a sensor sitting exactly on the target gave `0.0 / 0.0`.
            // Every comparison below is then false against the resulting
            // NaN and the range check passes vacuously, where upstream gets
            // `dp == 0`, `ang == pi/2` and violates. Measured in this
            // repo's oracle image at Eigen 3.4.0.
            let offset = world_to_target.translation.vector - world_to_sensor.translation.vector;
            let dir = offset.try_normalize(0.0).unwrap_or(offset);
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
    ///
    /// # Round 15/16/17: the pr2 115/2,201 depth mismatch, cause and residual
    ///
    /// Re-run fresh against the current tree and oracle (`moveit-diff
    /// --urdf crates/moveit-constraints/tests/fixtures/pr2.urdf --srdf
    /// crates/moveit-constraints/tests/fixtures/pr2.srdf --group right_arm
    /// --constraints 2000 --cases 100 --seed 4 --oracle
    /// tools/moveit-oracle/run-oracle.sh`, 2026-08-04): `cases: 2201,
    /// passed: 2086, failed: 115`, every failure a `visibility_cone`
    /// *distance* mismatch; `visibility_cone: 142 satisfied, 143 violated`
    /// on both sides, 0 boolean mismatches. Unchanged from the numbers this
    /// port's own history already recorded once mesh collision geometry
    /// landed (`moveit-model`/`moveit-collision` now retain and convert
    /// pr2's STL links) — so the mismatch is not the absence of mesh
    /// geometry.
    ///
    /// `decide_cone`'s own logic is not the cause: `cone_mesh` is a direct
    /// vertex/triangle transcription of upstream's `getVisibilityCone`
    /// (see that fn's own doc), `max_contacts: 1` here matches upstream's
    /// `req.max_contacts = 1` (`kinematic_constraint.cpp:1163`) exactly,
    /// and `allow_sensor_or_target_contact` matches `decideContact`
    /// exactly. If any of the three disagreed with upstream, the *verdict*
    /// would disagree too on at least some cases — it never does.
    ///
    /// Round 15 attributed the depth mismatch to pair-traversal-order
    /// tie-breaking under `max_contacts: 1`: with only the first robot-link
    /// contact *stored* (`moveit-collision`'s `accumulate_collision`,
    /// `stored_total < request.max_contacts` at `parry.rs:1120`), which
    /// link is "first" would depend on this crate's fixed
    /// `cross_pairs(&robot, &world)` order (`parry.rs:1324`) versus
    /// upstream's undocumented FCL BVH broadphase order, so two backends
    /// enumerating the same colliding set could pick two different "first"
    /// contacts whenever 2+ robot links touched the cone at once. That
    /// explanation was written as a falsifiable prediction rather than
    /// closed outright, specifically because nobody had measured whether
    /// pr2 cone placements actually produce such ties.
    ///
    /// Round 16 measured it directly against the 285-case sweep above,
    /// with [`VisibilityConstraint::cone_touching_link_count`]
    /// (`max_contacts: usize::MAX`, so every touching link is counted, not
    /// just the one `max_contacts: 1` stores) run against a temporary,
    /// git-reverted instrumentation patch to `tools/moveit-diff` correlating
    /// touching-link count with each case's real pass/fail verdict:
    ///
    /// | touching | n   | pass | fail |
    /// |----------|-----|------|------|
    /// | 0        | 142 | 142  | 0    |
    /// | 1        | 129 | 24   | 105  |
    /// | 2        | 13  | 4    | 9    |
    /// | 3        | 1   | 0    | 1    |
    ///
    /// The prediction ("all 115 failures are touching ≥ 2") is refuted for
    /// the dominant majority: 105/115 (91.3%) failures have `touching == 1`
    /// — one candidate pair, so `max_contacts: 1` storage picks it
    /// regardless of iteration order, ruling out a tie structurally, not
    /// just empirically. That majority's cause is `moveit-collision`'s
    /// already-documented deviation 6: independent penetration-depth
    /// *approximation* between this port's backend and upstream's FCL for
    /// the same single, unambiguous contact (case 104: oracle depth within
    /// 7ppm of `bl_caster_l_wheel_link`'s own cylinder radius, this port's
    /// is not; some near-zero cases disagree even in sign). That
    /// approximation lives in `moveit-collision`, owned by p3-acm — not
    /// fixable from this crate.
    ///
    /// The remaining 10/115 (8.7%) failures do have `touching >= 2` — a
    /// real tie for `max_contacts: 1` to break, unlike the majority above.
    /// p1-joints separately measured `tools/moveit-diff`'s own
    /// `visibility_cone_ambiguity_diagnostic` module (`d26916d`,
    /// `#[ignore]`d — needs `third_party/moveit_resources`, run with
    /// `cargo test -p moveit-diff --release
    /// visibility_cone_ambiguity_diagnostic:: -- --ignored --nocapture`),
    /// finding pr2's 17 parry-representable links touch the cone at most
    /// once each at pr2's *default* pose, and that case 104 specifically
    /// touches only one pair. Reproduced independently here with the same
    /// command, same result. That is a **different sample** from the sweep
    /// above (one fixed default pose versus 285 real random-pose cases) and
    /// does not by itself rule out ties among the sweep's other cases — an
    /// earlier revision of this comment (`f111dfb`) incorrectly used it to
    /// claim no ties exist anywhere in the 115, contradicting this crate's
    /// own 285-case measurement; that claim is retracted, not this one.
    ///
    /// Numeric evidence on whether the residual 10 are traversal-order- or
    /// deviation-6-caused: their magnitude distribution (n=10, range
    /// `[2.319e-4, 3.607e-3]`, mean `1.985e-3`) sits entirely inside the
    /// `touching == 1` failures' own range (n=105, range
    /// `[3.935e-5, 5.425e-2]`, mean `1.161e-2`) — 43/105 (41.0%) of the
    /// `touching == 1` failures fall in that same narrow band, and the
    /// residual 10's mean sits at only the 30th percentile of the
    /// `touching == 1` distribution. No separate cluster or characteristic
    /// magnitude distinguishes the `touching >= 2` failures from an
    /// ordinary sample of the deviation-6 family; nothing here positively
    /// supports treating them as traversal-order-caused. This is
    /// distributional evidence, not a per-pair confirmation (that would
    /// need oracle-side FCL pair instrumentation this crate has no access
    /// to), so it narrows rather than closes the residual: a real tie
    /// exists structurally for these 10, but every measurement made so far
    /// is at least as consistent with deviation 6 as with traversal order.
    ///
    /// Also: `touching >= 2` does not imply failure. 4/14 `touching >= 2`
    /// cases pass; the `touching >= 2` fail rate (10/14, 71.4%) is not
    /// higher than `touching == 1`'s (105/129, 81.4%) — touching count
    /// alone is a weak predictor of failure once `touching >= 1`.
    ///
    /// # Round 23: the residual 10 do not reproduce, and cannot under the
    /// current generator
    ///
    /// The 10 cases above have no recorded seed (`PORTING-PLAN.md` §148),
    /// so they cannot be re-drawn directly. What can be re-measured is the
    /// *mechanism*: a fresh sweep (`cargo run --release --example
    /// visibility_cone_depth_sweep -p moveit-constraints`, this crate's
    /// own independent oracle client, seeds 23/n=400 and 90210/n=2000,
    /// oracle stamp `c88557f4058892e9`) found **0/2400** `touching >= 2`
    /// cases — every near-placed case touches exactly one link, every
    /// far-placed case touches none, no exceptions.
    ///
    /// That is not sampling luck: the same binary's `--geometry-gaps`
    /// mode shows it is currently geometrically impossible. A near-placed
    /// cone's maximum reach from its own anchor (the hit link's shape
    /// center) is `max(target_radius, sensor_offset)` — the squared
    /// distance from a fixed point to a segment is convex in the segment
    /// parameter, so it maxes at an endpoint, not at the two legs'
    /// hypotenuse — capped at `0.015` by this generator's
    /// `rng.random_range(0.005..0.015)`
    /// (`tools/moveit-diff/src/main.rs:2078`). The tightest of pr2's 136
    /// eligible-link pairs (the four caster-wheel pairs) needs `0.0232` of
    /// reach to bridge; every pair needs more than the generator's max.
    /// `git log -S'0.005..0.015'`/`-S'radius="0.074792"'` confirm neither
    /// the generator's bounds nor pr2's fixture geometry have changed
    /// since before the 10-case measurement, so this is not drift — it is
    /// the same fact the 105-case sweep predates. `PORTING-PLAN.md` §164
    /// has the full derivation, the resulting contradiction with the
    /// (unreproducible) 10-case count, and `d26916d`'s independent,
    /// same-day corroboration via a different method (scene inspection of
    /// the actual 115 failures at `max_contacts: 64`, finding none
    /// ambiguous).
    ///
    /// With no `touching >= 2` case reachable to test, there is nothing
    /// left for traversal order to explain: deviation 6 alone accounts
    /// for every currently-reproducible `visibility_cone` mismatch. This
    /// closes the residual §121.2 left open, not merely narrows it
    /// further.
    pub fn decide(&self, state: &Posed) -> ConstraintEvaluationResult {
        let Some(result) = self.decide_by_angle(state) else {
            return self.decide_cone(state);
        };
        result
    }

    fn decide_cone(&self, state: &Posed) -> ConstraintEvaluationResult {
        let result = self.cone_collision_result(state, 1);

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

    /// Shared setup between [`VisibilityConstraint::decide_cone`] and
    /// [`VisibilityConstraint::cone_touching_link_count`]: build the cone,
    /// its throwaway local environment, and run the same collision check
    /// `decide_cone` runs, but with a caller-chosen `max_contacts` budget
    /// instead of `decide_cone`'s own hardcoded `1` (matching upstream's
    /// `req.max_contacts = 1`).
    fn cone_collision_result(
        &self,
        state: &Posed,
        max_contacts: usize,
    ) -> moveit_collision::CollisionResult {
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
            max_contacts,
            ..Default::default()
        };
        env.check_robot_collision(&request, state, &[], Some(&acm))
    }

    /// Diagnostic-only: the number of distinct robot links whose collision
    /// geometry touches the visibility cone at `state`, ignoring
    /// [`VisibilityConstraint::decide`]'s own `max_contacts: 1` storage
    /// budget entirely (`max_contacts: usize::MAX` here, so every
    /// disallowed contact gets a bucket, not just the first found).
    ///
    /// Not part of upstream's `VisibilityConstraint` API and not used by
    /// [`VisibilityConstraint::decide`] itself — `decide`'s own reported
    /// depth continues to come from whichever single contact
    /// `cone_collision_result(state, 1)` happens to find first, unchanged.
    ///
    /// This is the only tool in this repository that can re-measure the
    /// touching-link count of a real random-pose sweep case (as opposed to
    /// p1-joints' `tools/moveit-diff` diagnostic, which only covers pr2's
    /// *default* pose — see [`VisibilityConstraint::decide`]'s own doc
    /// comment for why that is a different sample, not a superset or
    /// substitute). Deleting this would make the ~10-case residual
    /// unmeasurable again from this tree.
    pub fn cone_touching_link_count(&self, state: &Posed) -> usize {
        self.cone_collision_result(state, usize::MAX)
            .contacts
            .map_or(0, |contacts| contacts.by_pair.len())
    }

    /// Diagnostic-only, §148's decisive test: every contact depth
    /// `cone_collision_result(state, usize::MAX)` finds, across every
    /// touching pair -- not just `decide_cone`'s own
    /// single reported depth (`cone_collision_result(state, 1)`'s first
    /// contact of whichever pair traversal order finds first). Order in the
    /// returned `Vec` follows `by_pair`'s `BTreeMap` iteration (link-name
    /// order), which is itself not `decide_cone`'s traversal order -- the
    /// point of this diagnostic is to give every candidate depth, so a
    /// caller can check "does the oracle's reported depth match *any*
    /// candidate", not to guess which candidate `decide_cone` would have
    /// picked.
    ///
    /// Not part of upstream's `VisibilityConstraint` API, and not used by
    /// [`VisibilityConstraint::decide`] itself, for the same reason
    /// [`VisibilityConstraint::cone_touching_link_count`] is not.
    pub fn cone_contact_depths(&self, state: &Posed) -> Vec<f64> {
        self.cone_collision_result(state, usize::MAX)
            .contacts
            .map(|contacts| {
                contacts
                    .by_pair
                    .values()
                    .flatten()
                    .map(|contact| contact.depth)
                    .collect()
            })
            .unwrap_or_default()
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

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::fs;

    use moveit_collision::AttachedBodyGeometry;
    use moveit_geometry::Cuboid;
    use moveit_model::MeshSearchPaths;
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;

    fn pr2_model() -> RobotModel {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/pr2.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/pr2.srdf");
        let urdf_xml = fs::read_to_string(urdf_path).expect("read pr2.urdf");
        let urdf = urdf_rs::read_file(urdf_path).expect("parse pr2.urdf");
        let srdf = SrdfModel::parse_file(srdf_path).expect("parse pr2.srdf");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("build pr2 model")
    }

    /// Settles, empirically rather than only by reading the C++, whether
    /// threading attached-body geometry through
    /// [`VisibilityConstraint::cone_collision_result`] (currently hardcoded
    /// `&[]`, see that fn's own doc comment) would change `decide()`'s
    /// verdict for a cone an attached body occludes. Bypasses the hardcoded
    /// `&[]` by calling `check_robot_collision` directly with a real
    /// [`AttachedBodyGeometry`], reusing this module's own
    /// `cone_mesh`/`allow_sensor_or_target_contact` -- the exact primitives
    /// `cone_collision_result` itself assembles.
    ///
    /// Upstream `decideContact` (`kinematic_constraint.cpp:1185-1189`)
    /// returns `true` (allowed, not a violation) for *any* contact where
    /// either body is `ROBOT_ATTACHED`, unconditionally -- no frame-name
    /// check, unlike the `ROBOT_LINK` branches right below it. So an
    /// attached body touching the visibility cone can never make upstream's
    /// `res.collision` true either: threading attached bodies through
    /// `cone_collision_result` would be a no-op for `decide()`'s output,
    /// not a fix for a live divergence.
    #[test]
    fn attached_body_touching_the_cone_is_not_an_occlusion() {
        let model = pr2_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_transform = posed.global_link_transform("base_bellow_link").unwrap();
        let link_pos = link_transform.translation.vector;
        let transforms = Transforms::new(model.model_frame()).unwrap();

        // Same 10m-away placement as `cone_far_from_the_robot_is_satisfied`
        // in tests/decide.rs, radius widened to 0.5 to match
        // `cone_through_a_robot_link_is_violated`'s cap-disc-through-a-box
        // geometry -- guaranteed clear of any real pr2 link.
        let world_to_sensor =
            Isometry3::translation(link_pos.x + 10.0, link_pos.y, link_pos.z + 1.0);
        let world_to_target = Isometry3::translation(link_pos.x + 10.0, link_pos.y, link_pos.z);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: world_to_sensor,
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: world_to_target,
            },
            8,
            VisibilityCriteria {
                target_radius: Some(0.5),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();

        // Sanity: with no attached body, this placement is satisfied --
        // nothing real already touches the cone at this location.
        assert_eq!(c.decide(&posed), ConstraintEvaluationResult::new(true, 0.0));

        // An attached body on `base_bellow_link`, positioned (in that
        // link's own frame) at the target's world point, so its box sits
        // inside the cone's filled base cap -- the same cap-through-a-box
        // geometry `cone_through_a_robot_link_is_violated` uses for a real
        // link.
        let shape_pose_in_link_frame = link_transform.inverse() * world_to_target;
        let shapes = vec![Arc::new(Shape::Cuboid(Cuboid::new(0.3, 0.3, 0.3).unwrap()))];
        let shape_poses = vec![shape_pose_in_link_frame];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "occluder",
            link_name: "base_bellow_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        // Reproduce `cone_collision_result`'s own env/ACM construction
        // exactly, but with a real attached body instead of its hardcoded
        // `&[]`.
        let cone = c.cone_mesh(world_to_sensor, world_to_target);
        let mut world = World::new();
        world.add_shape("cone", Arc::new(Shape::Mesh(cone)), Isometry3::identity());
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::new());
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_conditional_entry(
            "cone",
            allow_sensor_or_target_contact(
                c.sensor_frame().to_owned(),
                c.target_frame().to_owned(),
            ),
        );
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 1,
            ..Default::default()
        };

        // Confirm the attached body really does touch the cone once
        // included -- otherwise this test would pass for the wrong reason
        // (no contact at all, rather than an allowed one).
        let without_acm = env.check_robot_collision(&request, &posed, &[attached], None);
        assert!(
            without_acm.collision,
            "the attached body must actually overlap the cone for this test to mean anything"
        );

        let result = env.check_robot_collision(&request, &posed, &[attached], Some(&acm));
        assert!(
            !result.collision,
            "an attached body touching the cone must not be treated as an occlusion: \
             upstream decideContact (kinematic_constraint.cpp:1185-1189) allows any \
             ROBOT_ATTACHED contact unconditionally"
        );
    }
}
