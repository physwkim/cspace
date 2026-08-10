// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `geometry_msgs` <-> [`cspace_geometry`] primitive conversions.
//!
//! `cspace_geometry::{Isometry3, Vector3, UnitQuaternion}` are plain type
//! aliases to `nalgebra` types (`crates/cspace-geometry/src/lib.rs:216-223`),
//! not opaque local structs -- so on the core side, as on the `r2r` side,
//! every type this module converts is foreign to `cspace-ros`. Each `Msg*`
//! wrapper below exists solely so `impl TryFrom` has a locally-defined type
//! to attach to; see this crate's `lib.rs` doc comment for why that is
//! required at all (Rust's orphan rule) and not specific to `geometry_msgs`.
//!
//! `geometry_msgs::msg::Point` and `geometry_msgs::msg::Vector3` are
//! distinct wire types (position vs. direction) that both map onto the same
//! core type, `cspace_geometry::Vector3` -- the core crate has no separate
//! `Point`. This mirrors upstream: `moveit_core`'s own conversions go through
//! `tf2_eigen`, which likewise treats both as `Eigen::Vector3d`. Not a
//! narrowing introduced by this port.

use cspace_error::Error;
use cspace_geometry::{Isometry3, UnitQuaternion, Vector3 as CoreVector3};
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

/// Wraps `geometry_msgs::msg::Transform` (see module doc: orphan-rule wrapper).
#[derive(Debug, Clone, PartialEq)]
pub struct Transform(pub geometry_msgs::Transform);

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

    /// # The generic rule (round 15, PORTING-PLAN.md §211)
    ///
    /// This is the rule nine of this crate's ten Quaternion/Pose conversion
    /// sites reach -- every one except `OrientationConstraint.orientation`,
    /// which is not a `Quaternion`/`Pose` at this impl at all: it goes
    /// through [`OrientationConstraintQuaternion`] below instead, so this
    /// impl is never on that tenth site's path.
    ///
    /// Two upstream helpers reach here, and both apply the identical
    /// "normalize unconditionally, never fail" rule: `planning_scene.cpp`'s
    /// `utilities::poseMsgToEigen` (`:76-82`, own docstring: "normalizing
    /// the quaternion part if necessary") and `tf2_eigen.hpp`'s
    /// `fromMsg(const geometry_msgs::msg::Pose&, Eigen::Isometry3d&)`
    /// (`:493-505`) -- both call `quaternion.normalize()` unconditionally
    /// before building the isometry, no threshold, any norm. Round 14
    /// (`4ff563d`) mistook the *three* sites that reach the latter helper
    /// (`PositionConstraint::configure` `:405-406`/`:433-434`,
    /// `VisibilityConstraint::configure` `:845-846`/`:858-859`) for a third,
    /// stricter rule because each is followed by `ASSERT_ISOMETRY`, which
    /// reads like a second check. It is not one in the build that ships:
    /// `geometric_shapes/check_isometry.h` expands the macro to
    /// `(void)sizeof(transform);` under `NDEBUG` (confirmed against the
    /// header present in this project's ROS docker image, not from memory)
    /// and to a debug-only `assert` otherwise -- either way it runs *after*
    /// `fromMsg`'s own unconditional normalize and cannot change what value
    /// reached the constructor. Nine sites, one rule, not three; see
    /// PORTING-PLAN.md §211 for the full site table and the correction to
    /// its own first draft.
    ///
    /// The one input this rule truly cannot answer is exact-zero norm:
    /// Eigen's own `MatrixBase::normalize()` (`Eigen/src/Core/Dot.h:145-151`)
    /// guards with `if (z > RealScalar(0))`, so an all-zero quaternion is
    /// left unchanged by every path above, and the `Isometry3d` built from
    /// it downstream carries a zero (not unit) rotation matrix -- a value
    /// `nalgebra::UnitQuaternion` has no representation for at all
    /// (confirmed against the Eigen headers actually present in the ROS
    /// docker image, not from memory). D6 applies to that case alone (and
    /// to non-finite input, which has no upstream analogue either): reject,
    /// matching `zero_quaternion_is_rejected`/`nan_quaternion_is_rejected`
    /// below. Everything else -- including `norm == 2.0`, which `4ff563d`
    /// incorrectly started rejecting on this generic path before §211's
    /// ten-site sweep found the mistake -- upstream normalizes and this
    /// crate must too (`norm_far_from_one_is_renormalized_not_rejected`).
    fn try_from(msg: Quaternion) -> Result<Self, Self::Error> {
        let q = nalgebra::Quaternion::new(msg.0.w, msg.0.x, msg.0.y, msg.0.z);
        let norm = q.norm();
        if !norm.is_finite() || norm <= f64::EPSILON {
            return Err(Error::construct(format!(
                "geometry_msgs/Quaternion {{x: {}, y: {}, z: {}, w: {}}} has \
                 norm {norm}, too close to zero (or non-finite) to have a \
                 unit-norm representative. Upstream's own \
                 `Eigen::Quaterniond::normalize()` (Eigen/src/Core/Dot.h:145-151) \
                 leaves an exact-zero quaternion unchanged rather than \
                 failing, which this port cannot reproduce: nalgebra's \
                 UnitQuaternion has no zero-norm representation at all. A \
                 common cause: the field was left at its wire default \
                 (all-zero) instead of being set.",
                msg.0.x, msg.0.y, msg.0.z, msg.0.w
            )));
        }
        Ok(UnitQuaternion::new_normalize(q))
    }
}

/// Wraps `geometry_msgs::msg::Quaternion` specifically for
/// `OrientationConstraint.orientation` (round 15, PORTING-PLAN.md §211):
/// the *only* one of this crate's ten Quaternion/Pose conversion sites whose
/// upstream reaching path is `OrientationConstraint::configure`
/// (`kinematic_constraint.cpp:609-615`) rather than the generic
/// "normalize unconditionally" rule the other nine reach (see
/// `TryFrom<Quaternion> for UnitQuaternion` above). A distinct wrapper type,
/// not a threshold parameter bolted onto the generic conversion, so a call
/// site cannot silently apply the wrong upstream rule by passing the wrong
/// argument -- the type names which rule applies.
#[derive(Debug, Clone, PartialEq)]
pub struct OrientationConstraintQuaternion(pub geometry_msgs::Quaternion);

impl TryFrom<OrientationConstraintQuaternion> for UnitQuaternion {
    type Error = Error;

    /// # `OrientationConstraint::configure`'s own suspicion threshold
    /// (`kinematic_constraint.cpp:609-615`, round 15/§211)
    ///
    /// Upstream's own guard is `fabs(q.norm() - 1.0) > 1e-3`: anything
    /// further than that from unit norm (including all-zero and huge
    /// magnitudes alike) is "probably incorrect" and upstream **substitutes
    /// identity** for it, silently continuing rather than failing the whole
    /// constraint. Substituting identity for a caller-supplied constraint
    /// that upstream itself flags as almost certainly wrong is not
    /// "upstream defines what this defaults to" (D14's test, e.g.
    /// `weight: 0.0` -> `1.0`) -- there is no wire convention that an
    /// off-norm quaternion *means* identity, only a best-effort fallback for
    /// input upstream cannot trust. D6 applies: this crate rejects instead,
    /// with the guard matching upstream's own detection threshold
    /// (`|norm - 1.0| > 1e-3`) so it catches exactly what upstream itself
    /// calls suspicious -- no wider and no narrower. Inside that band
    /// (realistic wire rounding noise), the value is renormalized, not
    /// rejected -- `nalgebra`'s `UnitQuaternion::new_normalize` on a
    /// *near*-unit input reads the genuinely-intended rotation through
    /// float noise, matching upstream's own practical trust of anything
    /// that close.
    fn try_from(msg: OrientationConstraintQuaternion) -> Result<Self, Self::Error> {
        let q = nalgebra::Quaternion::new(msg.0.w, msg.0.x, msg.0.y, msg.0.z);
        let norm = q.norm();
        if !norm.is_finite() || (norm - 1.0).abs() > 1e-3 {
            return Err(Error::construct(format!(
                "OrientationConstraint.orientation {{x: {}, y: {}, z: {}, w: {}}} has \
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

    /// Always uses the generic rule (`TryFrom<Quaternion> for
    /// UnitQuaternion` above, PORTING-PLAN.md §211) -- every `Pose` this
    /// crate converts reaches an upstream helper that normalizes the
    /// embedded quaternion unconditionally, never
    /// `OrientationConstraint::configure`'s stricter suspicion rule (that
    /// field is a bare `Quaternion` on the wire, not a `Pose`, and is routed
    /// through [`OrientationConstraintQuaternion`] instead -- this impl is
    /// never on that path). Fails exactly when the embedded orientation's
    /// norm is exact-zero or non-finite (the generic rule's one
    /// unanswerable input, see above) -- **not** at `norm == 2.0` or any
    /// other off-norm-but-nonzero value, which the generic rule renormalizes
    /// (round 14's `4ff563d` briefly made this doc comment false by
    /// tightening the shared conversion without checking this impl's own
    /// callers; §211 is the fix). Position has no failure mode of its own.
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

impl TryFrom<Transform> for Isometry3 {
    type Error = Error;

    /// `geometry_msgs/Transform` is `Pose` with the two fields renamed
    /// (`translation`/`rotation` instead of `position`/`orientation`) and
    /// `translation` typed as `Vector3` rather than `Point` -- both of which
    /// this module already maps onto the same core types (module doc: the
    /// core crate has no separate `Point`). So this shares
    /// `TryFrom<Pose>`'s failure mode exactly: exact-zero or non-finite
    /// orientation norm, nothing else.
    ///
    /// Reached from `PlanningScene.fixed_frame_transforms` via
    /// `crate::scene::planning_scene`, upstream's
    /// `SceneTransforms::setTransforms` -> `Transforms::setTransforms`,
    /// which converts each entry with `tf2::fromMsg` -- the same
    /// `tf2_eigen` path `Pose` takes, not a stricter one.
    fn try_from(msg: Transform) -> Result<Self, Self::Error> {
        let translation = CoreVector3::try_from(Vector3(msg.0.translation))?;
        let rotation = UnitQuaternion::try_from(Quaternion(msg.0.rotation))?;
        Ok(Isometry3::from_parts(
            Translation3::from(translation),
            rotation,
        ))
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

    // PORTING-PLAN.md §211: the generic rule (this impl) has no 1e-3
    // threshold at all -- only exact-zero/non-finite norm is unanswerable.
    // Anything else, including values `OrientationConstraintQuaternion`'s
    // rule below would reject, is renormalized here, matching upstream's
    // unconditional `quaternion.normalize()`.

    #[test]
    fn norm_just_inside_orientation_rules_1e_minus_3_tolerance_is_also_accepted_here() {
        // PORTING-PLAN.md §215's per-site table: pairs with
        // `orientation_norm_just_inside_the_1e_minus_3_tolerance_is_accepted`
        // below -- this crate had never run the generic rule at exactly
        // norm=1.0009 before (only the strict rule was pinned at this exact
        // value); the generic rule's "no threshold at all" claim was
        // previously reasoned from the w=2.0/w=1.0011 tests, not run here.
        let msg = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0 + 0.0009, // |norm - 1.0| = 0.0009 < 1e-3
        });
        UnitQuaternion::try_from(msg).unwrap();
    }

    #[test]
    fn norm_just_outside_orientation_rules_1e_minus_3_tolerance_is_still_accepted_here() {
        let msg = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0 + 0.0011, // |norm - 1.0| = 0.0011 > 1e-3 -- rejected under
                             // OrientationConstraintQuaternion's rule below,
                             // accepted here: two different upstream rules.
        });
        UnitQuaternion::try_from(msg).unwrap();
    }

    #[test]
    fn norm_far_from_one_is_renormalized_not_rejected() {
        // §211: `4ff563d` (round 14) made this generic rule reject
        // norm=2.0, reasoning from `OrientationConstraint::configure`'s own
        // 1e-3 suspicion threshold -- but that threshold is a rule at ONE of
        // this impl's nine call sites (all reached through `Pose`, none of
        // them `OrientationConstraint.orientation`), not this generic rule's
        // own. The upstream helpers this rule actually mirrors
        // (`poseMsgToEigen`, `tf2_eigen.hpp`'s `fromMsg`) normalize any
        // nonzero finite quaternion unconditionally and never fail on norm
        // alone. See `OrientationConstraintQuaternion`'s own
        // `orientation_norm_far_from_one_is_rejected_not_silently_renormalized`
        // test below for the same input rejected under the *other* rule --
        // the pair is what proves the two rules are actually distinct.
        let msg = Quaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 2.0,
        });
        let q = UnitQuaternion::try_from(msg).unwrap();
        assert_relative_eq!(q.into_inner().norm(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn orientation_zero_quaternion_is_rejected() {
        let zero = OrientationConstraintQuaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        });
        let err = UnitQuaternion::try_from(zero).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn orientation_nan_quaternion_is_rejected() {
        let nan = OrientationConstraintQuaternion(geometry_msgs::Quaternion {
            x: f64::NAN,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        });
        let err = UnitQuaternion::try_from(nan).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn orientation_norm_just_inside_the_1e_minus_3_tolerance_is_accepted() {
        let msg = OrientationConstraintQuaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0 + 0.0009, // |norm - 1.0| = 0.0009 < 1e-3
        });
        UnitQuaternion::try_from(msg).unwrap();
    }

    #[test]
    fn orientation_norm_just_outside_the_1e_minus_3_tolerance_is_rejected() {
        let msg = OrientationConstraintQuaternion(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0 + 0.0011, // |norm - 1.0| = 0.0011 > 1e-3
        });
        let err = UnitQuaternion::try_from(msg).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn orientation_norm_far_from_one_is_rejected_not_silently_renormalized() {
        // Same input as `norm_far_from_one_is_renormalized_not_rejected`
        // above (the generic rule) -- here it must Err instead, pinning
        // that the two rules genuinely diverge on this value, not just in
        // their doc comments.
        let msg = OrientationConstraintQuaternion(geometry_msgs::Quaternion {
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

    #[test]
    fn pose_with_norm_2_orientation_succeeds_and_normalizes() {
        // §211: the regression `4ff563d` introduced and this round fixes --
        // a scene-side Pose (unlike an OrientationConstraint) must accept an
        // off-norm-but-nonzero orientation and normalize it, matching
        // upstream's `poseMsgToEigen`/`tf2_eigen::fromMsg`.
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
                w: 2.0,
            },
        });
        let iso = Isometry3::try_from(msg).unwrap();
        assert_relative_eq!(iso.rotation.into_inner().norm(), 1.0, epsilon = 1e-12);
    }
}
