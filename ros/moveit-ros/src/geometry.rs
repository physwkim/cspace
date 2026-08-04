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
    /// which has no unit-norm representative at all. `nalgebra`'s own
    /// `UnitQuaternion::new_normalize` would silently divide by zero (NaN)
    /// rather than error, so the zero/non-finite-norm case is checked here
    /// explicitly instead of delegated. A merely *near*-unit input (the
    /// normal case for any real wire value, given float rounding) is
    /// silently renormalized rather than rejected -- this matches upstream
    /// (`moveit_core`'s own msg->Eigen conversions normalize unconditionally
    /// via `Eigen::Quaterniond::normalized()`), and is not the kind of
    /// "failure absorbed into a silent default" D6 warns about: it is
    /// reading a genuinely-intended unit quaternion through its wire rounding
    /// noise, not substituting a default for missing/invalid data.
    fn try_from(msg: Quaternion) -> Result<Self, Self::Error> {
        let q = nalgebra::Quaternion::new(msg.0.w, msg.0.x, msg.0.y, msg.0.z);
        let norm = q.norm();
        if !norm.is_finite() || norm <= f64::EPSILON {
            return Err(Error::construct(format!(
                "geometry_msgs/Quaternion {{x: {}, y: {}, z: {}, w: {}}} has \
                 zero or non-finite norm ({norm}); cannot normalize to a unit \
                 quaternion. A common cause: the field was left at its wire \
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
