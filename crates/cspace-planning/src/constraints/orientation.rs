// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   (class OrientationConstraint)
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   (OrientationConstraint::configure, OrientationConstraint::decide,
//    calcEulerAngles, normalizeAbsoluteAngle)

use std::f64::consts::PI;

use cspace_core::error::{Error, Result};
use cspace_core::geometry::{Rotation3, Transforms, UnitQuaternion, Vector3};
use cspace_core::model::RobotModel;
use cspace_core::state::Posed;

use crate::constraints::ConstraintEvaluationResult;

const EPS: f64 = f64::EPSILON;

/// Normalizes an angle to `[-π, π]` and takes the absolute value, so the
/// result lies in `[0, π]`. Upstream `normalizeAbsoluteAngle`, ported
/// verbatim from `kinematic_constraint.cpp:83-87`.
fn normalize_absolute_angle(angle: f64) -> f64 {
    let normalized = angle.abs() % (2.0 * PI);
    (2.0 * PI - normalized).min(normalized)
}

/// Intrinsic XYZ Euler angles of `r`, and whether `r` is away from the
/// `pitch = ±π/2` singularity. Ported verbatim from
/// `kinematic_constraint.cpp:96-131` (itself copied upstream from Eigen's
/// unsupported `EulerSystem.h`), with `i=0, j=1, k=2` (the template's
/// `Derived` is always a plain `Matrix3d` at the one call site, so the
/// general index parameters collapse to these constants — matching
/// upstream's own instantiation).
fn calc_euler_angles(r: &Rotation3) -> (Vector3, bool) {
    let m = r.matrix();
    let rsum = ((m[(0, 0)] * m[(0, 0)]
        + m[(0, 1)] * m[(0, 1)]
        + m[(1, 2)] * m[(1, 2)]
        + m[(2, 2)] * m[(2, 2)])
        / 2.0)
        .sqrt();
    let mut res = Vector3::zeros();
    res[1] = m[(0, 2)].atan2(rsum);
    if rsum > 4.0 * f64::EPSILON {
        res[0] = (-m[(1, 2)]).atan2(m[(2, 2)]);
        res[2] = (-m[(0, 1)]).atan2(m[(0, 0)]);
        return (res, true);
    }
    if m[(0, 2)] > 0.0 {
        let spos = m[(1, 0)] + m[(2, 1)];
        let cpos = m[(1, 1)] - m[(2, 0)];
        res[0] = spos.atan2(cpos);
        res[2] = 0.0;
        return (res, false);
    }
    let sneg = m[(2, 1)] - m[(1, 0)];
    let cneg = m[(1, 1)] + m[(2, 0)];
    res[0] = sneg.atan2(cneg);
    res[2] = 0.0;
    (res, false)
}

/// How the three axis tolerances of an [`OrientationConstraint`] are
/// interpreted, and the rotation-error decomposition `decide()` uses to
/// compare against them.
///
/// # Deviation from upstream: one enum, not three floats plus a tag
///
/// `moveit_msgs::msg::OrientationConstraint` stores
/// `absolute_x_axis_tolerance`/`_y_`/`_z_` alongside a `parameterization`
/// `int32` (`XYZ_EULER_ANGLES` or `ROTATION_VECTOR`) that changes what those
/// three numbers *mean*: as XYZ Euler angles they are compared against a
/// roll/pitch/yaw decomposition of the rotation error (with a singularity
/// swap at `pitch ≈ ±π/2`, see `calc_euler_angles`); as a rotation vector
/// they are compared against the `|axis * angle|` components of the same
/// error, expressed in the constraint's own target frame. Same three
/// numbers, two incompatible readings selected by a sibling tag — this
/// port makes the tag and the numbers one value instead, so a caller can no
/// more build "`ROTATION_VECTOR` tolerances interpreted as Euler angles"
/// than the type system allows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OrientationTolerance {
    /// Compare against an intrinsic XYZ Euler angle decomposition of the
    /// rotation error. Has a singularity at `pitch ≈ ±π/2` (handled per
    /// `calc_euler_angles`'s doc comment); recommended only when
    /// subframes/attached bodies are not involved.
    XyzEuler {
        /// Tolerance on roll (about X), radians.
        x: f64,
        /// Tolerance on pitch (about Y), radians.
        y: f64,
        /// Tolerance on yaw (about Z), radians.
        z: f64,
    },
    /// Compare against the components of the rotation error's axis-angle
    /// (Rodrigues) vector, expressed in the constraint's target frame.
    /// Singularity-free; upstream recommends this parameterization.
    RotationVector {
        /// Tolerance about X, radians.
        x: f64,
        /// Tolerance about Y, radians.
        y: f64,
        /// Tolerance about Z, radians.
        z: f64,
    },
}

/// Where an [`OrientationConstraint`]'s target rotation is expressed, and
/// what `decide()` must therefore do to compare against it.
///
/// Same shape as `position::ReferenceFrame` and the same reason: upstream's
/// `mobile_frame_` flag changes not just how `desired_rotation_matrix_` was
/// computed but what `decide()` reads to get the live target (a cached
/// inverse, versus a fresh `getFrameTransform()` lookup composed with the
/// stored matrix) — one flag, two meanings, folded here into one enum whose
/// variants each carry exactly what their `decide()` branch needs.
#[derive(Debug, Clone, PartialEq)]
enum OrientationTarget {
    /// `frame` is fixed; `rotation_matrix_inv` is the target rotation's
    /// inverse, already expressed in `frame` and cached at construction
    /// (upstream `desired_rotation_matrix_inv_`).
    Fixed {
        frame: String,
        rotation_matrix_inv: Rotation3,
    },
    /// `frame` is mobile; `rotation_matrix` is the target rotation as given
    /// (upstream `desired_rotation_matrix_`, un-transformed in this branch)
    /// and must be composed with a fresh [`Posed::frame_transform`] lookup
    /// on every `decide()` call.
    Mobile {
        frame: String,
        rotation_matrix: Rotation3,
    },
}

/// Constrains a link's orientation to lie within per-axis tolerances of a
/// target quaternion.
///
/// Upstream `kinematic_constraints::OrientationConstraint`.
///
/// # Deviation from upstream: no "near-invalid quaternion" fallback
///
/// Upstream's `configure()` receives a raw `geometry_msgs::msg::Quaternion`
/// (four independent floats) and substitutes the identity when its norm is
/// more than `1e-3` from `1.0`. [`OrientationConstraint::new`] instead takes
/// a [`UnitQuaternion`], which `nalgebra` cannot construct in an unnormalized
/// state — the check upstream performs at runtime has no case left to catch
/// once the type itself guarantees the invariant.
///
/// # Deviation from upstream: an unresolvable frame is an error, not a
/// warning
///
/// Upstream's `configure()` only logs when `oc.header.frame_id` is empty and
/// proceeds anyway, silently building a constraint whose `decide()` (via
/// `getFrameTransform("")`) later resolves to the identity every time. This
/// port rejects an empty or unresolvable frame at construction, matching
/// [`crate::constraints::PositionConstraint`]'s equivalent check — `decide()` cannot
/// recover from either one, so surfacing it as a warning three calls
/// upstream of the actual failure only defers the same silent-wrong-answer
/// outcome.
#[derive(Debug, Clone, PartialEq)]
pub struct OrientationConstraint {
    link_index: usize,
    link_name: String,
    /// The raw target rotation in the constraint's own header frame, before
    /// any fixed-frame transform — used only by the
    /// [`OrientationTolerance::RotationVector`] branch of `decide()`, which
    /// upstream reads from `desired_R_in_frame_id_` regardless of whether
    /// the frame is mobile or fixed. Independent of [`OrientationTarget`]
    /// because upstream keeps it independent: it is not what the
    /// mobile/fixed split changes the meaning of.
    desired_r_in_frame_id: Rotation3,
    target: OrientationTarget,
    tolerance: OrientationTolerance,
    weight: f64,
}

impl OrientationConstraint {
    /// Build and resolve an orientation constraint against `model`.
    ///
    /// `weight <= f64::EPSILON` (including negative weights) normalizes to
    /// `1.0` rather than erroring — see
    /// [`crate::constraints::JointConstraint::new`]'s "Weight normalization" doc section
    /// for the full rationale and the D6/D14 boundary this is not D6.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `link_name` is not in `model`, or if
    /// `frame_id` is empty and not the model frame or a link name (see this
    /// type's doc comment).
    pub fn new(
        model: &RobotModel,
        tf: &Transforms,
        link_name: &str,
        frame_id: &str,
        orientation: UnitQuaternion,
        tolerance: OrientationTolerance,
        weight: f64,
    ) -> Result<Self> {
        let link = model.link_model(link_name)?;
        let weight = if weight <= EPS { 1.0 } else { weight };

        let desired_r_in_frame_id = orientation.to_rotation_matrix();

        let target = if tf.can_transform(frame_id) {
            let q_in_target = tf.transform_quaternion(frame_id, &orientation)?;
            OrientationTarget::Fixed {
                frame: tf.target_frame().to_string(),
                rotation_matrix_inv: q_in_target.to_rotation_matrix().inverse(),
            }
        } else {
            if !model.has_link_model(frame_id) && frame_id != model.model_frame() {
                return Err(Error::unknown_name("frame", frame_id));
            }
            OrientationTarget::Mobile {
                frame: frame_id.to_string(),
                rotation_matrix: desired_r_in_frame_id,
            }
        };

        let tolerance = match tolerance {
            OrientationTolerance::XyzEuler { x, y, z } => OrientationTolerance::XyzEuler {
                x: x.abs(),
                y: y.abs(),
                z: z.abs(),
            },
            OrientationTolerance::RotationVector { x, y, z } => {
                OrientationTolerance::RotationVector {
                    x: x.abs(),
                    y: y.abs(),
                    z: z.abs(),
                }
            }
        };

        Ok(Self {
            link_index: link.link_index(),
            link_name: link_name.to_string(),
            desired_r_in_frame_id,
            target,
            tolerance,
            weight,
        })
    }

    /// `getLinkModel` (name only — this crate resolves indices privately).
    pub fn link_name(&self) -> &str {
        &self.link_name
    }

    /// `getReferenceFrame`
    pub fn reference_frame(&self) -> &str {
        match &self.target {
            OrientationTarget::Fixed { frame, .. } | OrientationTarget::Mobile { frame, .. } => {
                frame
            }
        }
    }

    /// `mobileReferenceFrame`
    pub fn mobile_reference_frame(&self) -> bool {
        matches!(self.target, OrientationTarget::Mobile { .. })
    }

    /// Not an upstream accessor: needed by
    /// `crate::constraints::utils::update_orientation_constraint`, which reconstructs an
    /// `OrientationConstraint` from an existing one's fields rather than
    /// mutating a stored `moveit_msgs` field in place (see this crate's
    /// introducing doc comment on why `new()` replaces `configure()`).
    pub fn weight(&self) -> f64 {
        self.weight
    }

    /// See [`OrientationConstraint::weight`].
    pub fn tolerance(&self) -> OrientationTolerance {
        self.tolerance
    }

    /// `getDesiredRotationMatrixInRefFrame`: the target rotation as given at
    /// construction, in the constraint's own header frame
    /// (`frame_id`/[`OrientationConstraint::reference_frame`] before any
    /// fixed-frame transform is applied) — upstream's own doc comment notes
    /// this is unaffected by the mobile/fixed split, matching this port's
    /// own `desired_r_in_frame_id` field, which is set once and never
    /// touched by either `OrientationTarget` branch.
    pub fn desired_rotation_matrix_in_ref_frame(&self) -> Rotation3 {
        self.desired_r_in_frame_id
    }

    /// `getDesiredRotationMatrix`: the target rotation expressed in
    /// [`OrientationConstraint::reference_frame`] — for the `Mobile` branch
    /// this is the same untransformed value as
    /// [`OrientationConstraint::desired_rotation_matrix_in_ref_frame`]
    /// (upstream never transforms it in that branch either); for the
    /// `Fixed` branch it is the transform-composed rotation
    /// cached at construction, recovered here as the inverse of
    /// `rotation_matrix_inv` (upstream's own `desired_rotation_matrix_inv_ =
    /// desired_rotation_matrix_.transpose()`, and a rotation matrix's
    /// inverse and transpose are the same matrix) rather than storing the
    /// same rotation twice.
    pub fn desired_rotation_matrix(&self) -> Rotation3 {
        match &self.target {
            OrientationTarget::Fixed {
                rotation_matrix_inv,
                ..
            } => rotation_matrix_inv.inverse(),
            OrientationTarget::Mobile {
                rotation_matrix, ..
            } => *rotation_matrix,
        }
    }

    /// `OrientationConstraint::decide`.
    pub fn decide(&self, state: &Posed) -> ConstraintEvaluationResult {
        let actual = state
            .global_link_transform_at(self.link_index)
            .rotation
            .to_rotation_matrix();

        let diff = match &self.target {
            OrientationTarget::Fixed {
                rotation_matrix_inv,
                ..
            } => rotation_matrix_inv * actual,
            OrientationTarget::Mobile {
                frame,
                rotation_matrix,
            } => {
                let frame_r = state
                    .frame_transform(frame)
                    .expect("mobile reference frame was validated resolvable at construction")
                    .rotation
                    .to_rotation_matrix();
                let tmp = frame_r * rotation_matrix;
                tmp.inverse() * actual
            }
        };

        let (x_tol, y_tol, z_tol) = match self.tolerance {
            OrientationTolerance::XyzEuler { x, y, z } => (x, y, z),
            OrientationTolerance::RotationVector { x, y, z } => (x, y, z),
        };

        let xyz_rotation = match self.tolerance {
            OrientationTolerance::XyzEuler { .. } => {
                let (mut angles, away_from_singularity) = calc_euler_angles(&diff);
                if !away_from_singularity && normalize_absolute_angle(angles[0]) > z_tol + EPS {
                    angles[2] = angles[0];
                    angles[0] = 0.0;
                }
                angles.map(normalize_absolute_angle)
            }
            OrientationTolerance::RotationVector { .. } => {
                (self.desired_r_in_frame_id * diff.scaled_axis()).map(f64::abs)
            }
        };

        let satisfied = xyz_rotation[2] < z_tol + EPS
            && xyz_rotation[1] < y_tol + EPS
            && xyz_rotation[0] < x_tol + EPS;

        ConstraintEvaluationResult::new(
            satisfied,
            self.weight * (xyz_rotation[0] + xyz_rotation[1] + xyz_rotation[2]),
        )
    }
}
