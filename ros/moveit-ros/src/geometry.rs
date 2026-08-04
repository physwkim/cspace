// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `geometry_msgs` <-> [`moveit_geometry`] primitive conversions.
//!
//! `moveit_geometry::{Isometry3, Vector3, UnitQuaternion}` are plain type
//! aliases to `nalgebra` types (`crates/moveit-geometry/src/lib.rs:216-223`),
//! not opaque local structs -- so on the core side, as on the `r2r` side,
//! every type this module converts is foreign to `moveit-ros`. Each `Msg*`
//! wrapper below exists solely so `impl TryFrom` has a locally-defined type
//! to attach to; see this crate's `lib.rs` doc comment for why that is
//! required at all (Rust's orphan rule) and not specific to `geometry_msgs`.
//!
//! `geometry_msgs::msg::Point` and `geometry_msgs::msg::Vector3` are
//! distinct wire types (position vs. direction) that both map onto the same
//! core type, `moveit_geometry::Vector3` -- the core crate has no separate
//! `Point`. This mirrors upstream: `moveit_core`'s own conversions go through
//! `tf2_eigen`, which likewise treats both as `Eigen::Vector3d`. Not a
//! narrowing introduced by this port.

use moveit_error::Error;
use moveit_geometry::{Isometry3, UnitQuaternion, Vector3 as CoreVector3};
use nalgebra::Translation3;
use r2r::geometry_msgs::msg as geometry_msgs;

/// Wraps `geometry_msgs::msg::Point` (see module doc: orphan-rule wrapper).
#[derive(Debug, Clone, PartialEq)]
pub struct Point(pub geometry_msgs::Point);

/// Wraps `geometry_msgs::msg::Vector3` (see module doc: orphan-rule wrapper).
#[derive(Debug, Clone, PartialEq)]
pub struct Vector3(pub geometry_msgs::Vector3);

/// Wraps `geometry_msgs::msg::Quaternion` (see module doc: orphan-rule wrapper).
#[derive(Debug, Clone, PartialEq)]
pub struct Quaternion(pub geometry_msgs::Quaternion);

/// Wraps `geometry_msgs::msg::Pose` (see module doc: orphan-rule wrapper).
#[derive(Debug, Clone, PartialEq)]
pub struct Pose(pub geometry_msgs::Pose);

impl TryFrom<Point> for CoreVector3 {
    type Error = Error;

    /// Total in practice (`x`/`y`/`z` are unconstrained `f64`, same as the
    /// core type) -- fallible only to keep a uniform `TryFrom` surface
    /// across this module (D6), not because a failure case exists here.
    fn try_from(msg: Point) -> Result<Self, Self::Error> {
        Ok(CoreVector3::new(msg.0.x, msg.0.y, msg.0.z))
    }
}

impl TryFrom<CoreVector3> for Point {
    type Error = Error;

    fn try_from(v: CoreVector3) -> Result<Self, Self::Error> {
        Ok(Point(geometry_msgs::Point {
            x: v.x,
            y: v.y,
            z: v.z,
        }))
    }
}

impl TryFrom<Vector3> for CoreVector3 {
    type Error = Error;

    /// Total, same reasoning as `Point`'s impl above.
    fn try_from(msg: Vector3) -> Result<Self, Self::Error> {
        Ok(CoreVector3::new(msg.0.x, msg.0.y, msg.0.z))
    }
}

impl TryFrom<CoreVector3> for Vector3 {
    type Error = Error;

    fn try_from(v: CoreVector3) -> Result<Self, Self::Error> {
        Ok(Vector3(geometry_msgs::Vector3 {
            x: v.x,
            y: v.y,
            z: v.z,
        }))
    }
}

impl TryFrom<Quaternion> for UnitQuaternion {
    type Error = Error;

    /// The one genuine failure case in this module. `geometry_msgs/Quaternion`
    /// can represent any four `f64`s, including the all-zero wire default
    /// (`Default::default()`, e.g. an unset field in a larger message) --
    /// which has no unit-norm representative at all.
    ///
    /// # Divergence from upstream, round 14 (D6 vs. D14's "upstream defines
    /// a meaning" test, `kinematic_constraint.cpp:609-615`)
    ///
    /// Upstream's own guard is `fabs(q.norm() - 1.0) > 1e-3`: anything
    /// further than that from unit norm (including all-zero and huge
    /// magnitudes alike) is "probably incorrect" and upstream **substitutes
    /// identity** for it, silently continuing rather than failing the whole
    /// constraint. Before this round, this crate's own threshold was much
    /// narrower -- reject only `norm <= f64::EPSILON` or non-finite -- so a
    /// quaternion upstream itself would already call "probably incorrect"
    /// (e.g. `norm == 2.0`, `x/y/z/w` all `10.0`) fell *inside* the accepted
    /// range here and was silently renormalized with no warning at all,
    /// which is a narrower version of exactly the D6 shape (an
    /// unrecoverable/untrustworthy input answered anyway) that upstream's
    /// *own* identity substitution already is one instance of (see §183's
    /// `getFrameTransform` precedent, the same shape one level up).
    /// Substituting identity for a caller-supplied constraint that upstream
    /// itself flags as almost certainly wrong is not "upstream defines what
    /// this defaults to" (D14's test, e.g. `weight: 0.0` -> `1.0`) -- there
    /// is no wire convention that an off-norm quaternion *means* identity,
    /// only a best-effort fallback for input upstream cannot trust. D6
    /// applies: this crate rejects instead, but the guard now matches
    /// upstream's own detection threshold (`|norm - 1.0| > 1e-3`) instead of
    /// the much looser zero/non-finite-only check, so it actually catches
    /// what upstream itself calls suspicious. Inside that band (realistic
    /// wire rounding noise), the value is renormalized, not rejected --
    /// `nalgebra`'s `UnitQuaternion::new_normalize` on a *near*-unit input
    /// reads the genuinely-intended rotation through float noise, matching
    /// upstream's own practical trust of anything that close.
    fn try_from(msg: Quaternion) -> Result<Self, Self::Error> {
        let q = nalgebra::Quaternion::new(msg.0.w, msg.0.x, msg.0.y, msg.0.z);
        let norm = q.norm();
        if !norm.is_finite() || (norm - 1.0).abs() > 1e-3 {
            return Err(Error::construct(format!(
                "geometry_msgs/Quaternion {{x: {}, y: {}, z: {}, w: {}}} has \
                 norm {norm}, more than 1e-3 from 1.0 (or non-finite); \
                 upstream's own `OrientationConstraint::configure` \
                 (kinematic_constraint.cpp:609) calls this \"probably \
                 incorrect\" and substitutes identity -- this port rejects \
                 instead (D6: an untrustworthy input, not a documented wire \
                 default). A common cause: the field was left at its wire \
                 default (all-zero) instead of being set.",
                msg.0.x, msg.0.y, msg.0.z, msg.0.w
            )));
        }
        Ok(UnitQuaternion::new_normalize(q))
    }
}

impl TryFrom<UnitQuaternion> for Quaternion {
    type Error = Error;

    /// Total: every `UnitQuaternion` already carries unit norm by
    /// construction (`nalgebra`'s own invariant), so this direction can
    /// never fail. Still `TryFrom`, not `From`, for the uniform surface (D6).
    fn try_from(q: UnitQuaternion) -> Result<Self, Self::Error> {
        let inner = q.into_inner();
        Ok(Quaternion(geometry_msgs::Quaternion {
            x: inner.i,
            y: inner.j,
            z: inner.k,
            w: inner.w,
        }))
    }
}

impl TryFrom<Pose> for Isometry3 {
    type Error = Error;

    /// Fails exactly when the embedded orientation does
    /// (`Quaternion::try_from`'s zero/non-finite-norm case) -- position has
    /// no failure mode of its own.
    fn try_from(msg: Pose) -> Result<Self, Self::Error> {
        let translation = CoreVector3::try_from(Point(msg.0.position))?;
        let rotation = UnitQuaternion::try_from(Quaternion(msg.0.orientation))?;
        Ok(Isometry3::from_parts(
            Translation3::from(translation),
            rotation,
        ))
    }
}

impl TryFrom<Isometry3> for Pose {
    type Error = Error;

    /// Total: `Isometry3`'s rotation is always a valid `UnitQuaternion`.
    fn try_from(iso: Isometry3) -> Result<Self, Self::Error> {
        let position = Point::try_from(CoreVector3::from(iso.translation.vector))?.0;
        let orientation = Quaternion::try_from(iso.rotation)?.0;
        Ok(Pose(geometry_msgs::Pose {
            position,
            orientation,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn point_and_vector3_msgs_map_to_the_same_core_type() {
        let p = Point(geometry_msgs::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        });
        let v = Vector3(geometry_msgs::Vector3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        });
        assert_eq!(
            CoreVector3::try_from(p).unwrap(),
            CoreVector3::try_from(v).unwrap()
        );
    }

    #[test]
    fn zero_quaternion_is_rejected() {
        let zero = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        });
        let err = UnitQuaternion::try_from(zero).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn nan_quaternion_is_rejected() {
        let nan = Quaternion(geometry_msgs::Quaternion {
            x: f64::NAN,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        });
        let err = UnitQuaternion::try_from(nan).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn near_unit_quaternion_is_renormalized_not_rejected() {
        // 1e-9 off unit norm -- realistic wire rounding noise, not a
        // deliberately invalid input.
        let noisy = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0 + 1e-9,
        });
        let q = UnitQuaternion::try_from(noisy).unwrap();
        assert_relative_eq!(q.into_inner().norm(), 1.0, epsilon = 1e-12);
    }

    // PORTING-PLAN.md round 14: upstream's own suspicion threshold is
    // `fabs(norm - 1.0) > 1e-3` (`kinematic_constraint.cpp:609`) -- boundary
    // values on either side of it, not narrative scenarios.

    #[test]
    fn norm_just_inside_the_1e_minus_3_tolerance_is_accepted() {
        let msg = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0 + 0.0009, // |norm - 1.0| = 0.0009 < 1e-3
        });
        UnitQuaternion::try_from(msg).unwrap();
    }

    #[test]
    fn norm_just_outside_the_1e_minus_3_tolerance_is_rejected() {
        let msg = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0 + 0.0011, // |norm - 1.0| = 0.0011 > 1e-3
        });
        let err = UnitQuaternion::try_from(msg).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn norm_far_from_one_is_rejected_not_silently_renormalized() {
        // Before round 14 this crate only rejected zero/non-finite norm, so
        // norm=2.0 (which upstream's own `configure()` already calls
        // "probably incorrect" and replaces with identity) was silently
        // renormalized here with no warning at all -- a narrower version of
        // the same "unrecoverable input answered anyway" shape D6 exists to
        // prevent, and narrower than upstream's own detection band besides.
        let msg = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 2.0,
        });
        let err = UnitQuaternion::try_from(msg).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn point_round_trips_through_msg() {
        let original = CoreVector3::new(1.5, -2.25, 3.75);
        let msg = Point::try_from(original).unwrap();
        assert_eq!(msg.0.x, 1.5);
        assert_eq!(msg.0.y, -2.25);
        assert_eq!(msg.0.z, 3.75);
        let back = CoreVector3::try_from(msg).unwrap();
        assert_eq!(original, back);
    }

    #[test]
    fn vector3_round_trips_through_msg() {
        let original = CoreVector3::new(-4.5, 5.125, -6.0);
        let msg = Vector3::try_from(original).unwrap();
        assert_eq!(msg.0.x, -4.5);
        assert_eq!(msg.0.y, 5.125);
        assert_eq!(msg.0.z, -6.0);
        let back = CoreVector3::try_from(msg).unwrap();
        assert_eq!(original, back);
    }

    #[test]
    fn quaternion_round_trips_through_msg() {
        let original = UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let msg = Quaternion::try_from(original).unwrap();
        let back = UnitQuaternion::try_from(msg).unwrap();
        assert_relative_eq!(
            original.into_inner().coords,
            back.into_inner().coords,
            epsilon = 1e-12
        );
    }

    #[test]
    fn pose_round_trips_through_msg() {
        let original = Isometry3::from_parts(
            Translation3::new(1.0, -2.0, 0.5),
            UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3),
        );
        let msg = Pose::try_from(original).unwrap();
        let back = Isometry3::try_from(msg).unwrap();
        assert_relative_eq!(
            original.translation.vector,
            back.translation.vector,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            original.rotation.into_inner().coords,
            back.rotation.into_inner().coords,
            epsilon = 1e-12
        );
    }

    #[test]
    fn pose_with_degenerate_orientation_fails() {
        let msg = Pose(geometry_msgs::Pose {
            position: geometry_msgs::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
        });
        let err = Isometry3::try_from(msg).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }
}
