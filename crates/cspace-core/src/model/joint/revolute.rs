// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/revolute_joint_model.hpp
//   moveit_core/robot_model/src/revolute_joint_model.cpp

use std::f64::consts::PI;

use crate::geometry::{Isometry3, UnitQuaternion, Vector3};

use super::bounds::VariableBounds;

/// A revolute joint: one degree of freedom, rotation about a fixed axis.
///
/// Upstream `moveit::core::RevoluteJointModel`. The axis and the `continuous`
/// flag live here; the joint's single [`VariableBounds`] lives in the
/// owning [`crate::model::joint::JointModel::variable_bounds`], because
/// `set_continuous` (upstream `setContinuous`) mutates that bound as a side
/// effect and both must move together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevoluteJoint {
    axis: Vector3,
    continuous: bool,
}

impl Default for RevoluteJoint {
    /// Matches upstream's constructor: zero axis, not continuous. A zero
    /// axis is degenerate; callers are expected to call
    /// [`RevoluteJoint::set_axis`] before use, mirroring upstream's
    /// construct-then-`setAxis` sequence.
    fn default() -> Self {
        Self {
            axis: Vector3::zeros(),
            continuous: false,
        }
    }
}

impl RevoluteJoint {
    /// The axis of rotation: unit length after a non-degenerate
    /// [`RevoluteJoint::set_axis`], zero if never set (see
    /// [`RevoluteJoint::default`]) or set from a zero vector.
    pub fn axis(&self) -> Vector3 {
        self.axis
    }

    /// Set the axis of rotation. Upstream `RevoluteJointModel::setAxis`,
    /// which normalizes.
    ///
    /// `Eigen::Vector3d::normalized()` guards its zero-norm case (returns
    /// the input unchanged rather than dividing by zero — see
    /// `MatrixBase::normalized`'s `if (z > 0) ... else return n;` in
    /// `Dot.h`); nalgebra's `Vector3::normalize` has no such guard and
    /// divides by zero unconditionally, turning a zero axis into
    /// `[NaN, NaN, NaN]`. `try_normalize` with the upstream guard spelled
    /// out reproduces the exact contract: a zero axis (reachable
    /// unscreened from an explicit `<axis xyz="0 0 0"/>` in URDF) is left
    /// as zero, not turned into NaN that silently propagates through
    /// `computeTransform` into every downstream forward-kinematics
    /// consumer.
    ///
    /// `try_normalize`'s `min_norm` argument is `0.0` to match Eigen's
    /// exact `z > 0` guard, not merely "small": `if n <= min_norm { None }`
    /// (nalgebra) versus `if (z > 0) ... else ...` (Eigen) agree bit for
    /// bit at the zero boundary.
    pub fn set_axis(&mut self, axis: Vector3) {
        self.axis = axis.try_normalize(0.0).unwrap_or(axis);
    }

    /// Whether this joint wraps around (no position limit, `interpolate` and
    /// `distance` take the shorter way around the circle).
    pub fn is_continuous(&self) -> bool {
        self.continuous
    }

    /// Set the `continuous` flag directly, without touching bounds.
    ///
    /// Only [`crate::model::joint::JointModel::set_continuous`] calls this — it
    /// additionally mutates the joint's [`VariableBounds`], which live on
    /// the owning `JointModel`, not here (see this type's doc comment).
    pub(super) fn set_continuous_flag(&mut self, flag: bool) {
        self.continuous = flag;
    }

    pub(super) fn default_position(bounds: &VariableBounds) -> f64 {
        if bounds.min_position <= 0.0 && bounds.max_position >= 0.0 {
            0.0
        } else {
            (bounds.min_position + bounds.max_position) / 2.0
        }
    }

    /// Upstream: `variable_bounds_[0].max_position -
    /// variable_bounds_[0].min_position` -- ignores the `other_bounds`
    /// parameter entirely (upstream's own signature comments it out:
    /// `getMaximumExtent(const Bounds& /*other_bounds*/)`), unlike
    /// Prismatic/Planar/Floating, whose siblings all read `other_bounds` in
    /// some form. `bounds` here is always this joint's *own* installed
    /// bounds; the dispatcher must pass that, not its `other_bounds`
    /// argument.
    pub(super) fn maximum_extent(bounds: &VariableBounds) -> f64 {
        bounds.max_position - bounds.min_position
    }

    pub(super) fn satisfies_position_bounds(
        &self,
        value: f64,
        bounds: &VariableBounds,
        margin: f64,
    ) -> bool {
        if self.continuous {
            true
        } else {
            value >= bounds.min_position - margin && value <= bounds.max_position + margin
        }
    }

    /// Bring `*value` into `[-pi, pi]` by adding/subtracting multiples of
    /// `2*pi`, for a continuous joint; clamp to `bounds` otherwise. Always
    /// returns `true`, matching upstream (which returns an unconditional
    /// `true` for this joint type, unlike the other joint kinds).
    pub(super) fn enforce_position_bounds(&self, value: &mut f64, bounds: &VariableBounds) -> bool {
        if self.continuous {
            if *value <= -PI || *value > PI {
                *value %= 2.0 * PI;
                if *value <= -PI {
                    *value += 2.0 * PI;
                } else if *value > PI {
                    *value -= 2.0 * PI;
                }
            }
        } else if *value < bounds.min_position {
            *value = bounds.min_position;
        } else if *value > bounds.max_position {
            *value = bounds.max_position;
        }
        true
    }

    /// Add/subtract multiples of `2*pi` to bring `*value` back into
    /// `bounds`. Upstream applies this regardless of the `continuous` flag —
    /// it operates purely on `bounds`, so it is a no-op whenever `*value`
    /// is already inside them.
    pub(super) fn harmonize_position(value: &mut f64, bounds: &VariableBounds) -> bool {
        let mut modified = false;
        if *value < bounds.min_position {
            while *value + 2.0 * PI <= bounds.max_position {
                *value += 2.0 * PI;
                modified = true;
            }
        } else if *value > bounds.max_position {
            while *value - 2.0 * PI >= bounds.min_position {
                *value -= 2.0 * PI;
                modified = true;
            }
        }
        modified
    }

    pub(super) fn interpolate(&self, from: f64, to: f64, t: f64) -> f64 {
        if self.continuous {
            let diff = to - from;
            if diff.abs() <= PI {
                from + diff * t
            } else {
                let diff = if diff > 0.0 {
                    2.0 * PI - diff
                } else {
                    -2.0 * PI - diff
                };
                let mut state = from - diff * t;
                if state > PI {
                    state -= 2.0 * PI;
                } else if state < -PI {
                    state += 2.0 * PI;
                }
                state
            }
        } else {
            from + (to - from) * t
        }
    }

    pub(super) fn distance(&self, value1: f64, value2: f64) -> f64 {
        if self.continuous {
            let d = (value1 - value2).abs() % (2.0 * PI);
            if d > PI { 2.0 * PI - d } else { d }
        } else {
            (value1 - value2).abs()
        }
    }

    /// Rotation by `value` about [`RevoluteJoint::axis`].
    ///
    /// Upstream hand-expands the Rodrigues rotation matrix into the
    /// isometry's raw column-major storage (with the simpler
    /// `Eigen::Isometry3d(Eigen::AngleAxisd(value, axis_))` form left in a
    /// comment — never compiled, `revolute_joint_model.cpp:286`). This port
    /// uses that simpler, uncompiled form instead: nalgebra's axis-angle
    /// construction, not hand-rolled matrix coefficients. The two forms
    /// are equivalent *only* for a genuine unit axis — see below for the
    /// degenerate case, where they are not.
    ///
    /// # Degenerate axis: a disclosed divergence, not an oversight
    ///
    /// Upstream's compiled `computeTransform` (`revolute_joint_model.cpp:250-287`)
    /// is fed by `setAxis`'s precomputed products (`x2_`, `xy_`, ...,
    /// `:73-81`) and by the constructor's own initializer (`axis_(0,0,0)`,
    /// `:49`) — both leave `axis_ == (0,0,0)` with every product exactly
    /// `0.0` when the axis is degenerate. With every product zero, every
    /// off-diagonal matrix term vanishes and every diagonal term collapses
    /// to `t*0 + c = c` (`:266/272/278`), so upstream's matrix is exactly
    /// `cos(value)*I`. That is a rotation only at `value == 0`
    /// (`cos(0) == 1`); for any other `value` its determinant is
    /// `cos^3(value) != 1` and it is not orthogonal — upstream returns a
    /// non-rotation matrix here, an accident of C++ having no unit-vector
    /// type enforcement to stop it, not a contract this port owes
    /// bit-for-bit.
    ///
    /// `nalgebra::Isometry3` is translation composed with a
    /// [`UnitQuaternion`] and cannot represent a non-orthonormal "rotation"
    /// at all. `UnitQuaternion::from_axis_angle(&Unit::new_unchecked(axis),
    /// value)` on a zero `axis` does not panic and does not produce NaN —
    /// it silently returns a `UnitQuaternion` whose own promised unit-norm
    /// invariant is false (measured: norm `0.9553364...` at `value = 0.3`),
    /// which every downstream consumer of that "unit" quaternion then
    /// silently trusts.
    ///
    /// Since this port cannot reproduce upstream's raw non-rotation output
    /// through this type, it substitutes the identity rotation for a
    /// degenerate axis, regardless of `value`. "Rotate by any angle about
    /// no axis" is physically undefined; identity is the least-wrong
    /// finite answer a genuinely-unit-typed API can give. This agrees with
    /// upstream exactly at `value == 0` (both sides give identity) and
    /// diverges for `value != 0` by design. Do not "fix" this back to an
    /// unconditional `from_axis_angle` without first solving the
    /// representability problem above.
    ///
    /// [`RevoluteJoint::compute_variable_position`] needs no equivalent
    /// guard: its `axis_val`/`q_val` fold already degrades gracefully on a
    /// zero (or NaN) axis component via ordinary division, returning a
    /// NaN angle rather than requiring a distinct output shape the way
    /// this function does.
    pub(super) fn compute_transform(&self, value: f64) -> Isometry3 {
        if self.axis == Vector3::zeros() {
            return Isometry3::from_parts(
                nalgebra::Translation3::identity(),
                UnitQuaternion::identity(),
            );
        }
        Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_unchecked(self.axis), value),
        )
    }

    /// Recover the rotation angle about [`RevoluteJoint::axis`] from a
    /// transform produced by [`RevoluteJoint::compute_transform`].
    ///
    /// Upstream picks the axis component with the largest absolute value to
    /// avoid dividing by a near-zero component
    /// (`axis_.array().abs().maxCoeff(&max_idx)`,
    /// `revolute_joint_model.cpp:295`). At length 3 `Eigen::maxCoeff` does
    /// not vectorize, so — unlike the general no-argument-reduction family,
    /// whose NaN behavior *is* build-dependent at larger lengths — it has
    /// one exact, build-stable rule here (measured across `-O0`, `-O2`, and
    /// `-O2 -DEIGEN_DONT_VECTORIZE` against this repo's Eigen 3.4.0 oracle
    /// image): a left fold from index 0, `res = coeff(0)`, then `if (x >
    /// res) { res = x; idx = i }` per following index. A NaN component
    /// never displaces the incumbent (`x > res` is false when `x` is NaN),
    /// and a *leading* NaN is never displaced either (every later `x >
    /// NaN` is also false too) — so the fold returns a value rather than
    /// panicking on a NaN axis component, and a tie keeps the *first*
    /// index, not the last.
    ///
    /// `Iterator::reduce`'s `if cur.0.abs() > best.0.abs() { cur } else {
    /// best }` reproduces both measured properties; `Iterator::max_by`
    /// (the previous spelling here) does not: its `partial_cmp(...)
    /// .unwrap()` panics outright on a NaN component (reachable through
    /// [`RevoluteJoint::set_axis`] on a NaN input, which propagates rather
    /// than guards, unlike the zero-axis case — see `set_axis`'s doc
    /// comment), and even setting the panic aside, `max_by` keeps the
    /// *last* tied element, the opposite of Eigen's first-index rule.
    /// Every axis in the panda and fanuc fixtures is a single unit basis
    /// vector, so no tie is exercised by either oracle-backed fixture.
    pub(super) fn compute_variable_position(&self, transform: &Isometry3) -> f64 {
        let q = transform.rotation.quaternion();
        let components = [(self.axis.x, q.i), (self.axis.y, q.j), (self.axis.z, q.k)];
        let (axis_val, q_val) = components
            .into_iter()
            .reduce(|best, cur| {
                if cur.0.abs() > best.0.abs() {
                    cur
                } else {
                    best
                }
            })
            .expect("axis has three components");
        2.0 * (q_val / axis_val).atan2(q.w)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    fn bounded() -> (RevoluteJoint, VariableBounds) {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 1.0));
        let bounds = VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            position_bounded: true,
            ..Default::default()
        };
        (joint, bounds)
    }

    fn continuous() -> (RevoluteJoint, VariableBounds) {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 1.0));
        joint.continuous = true;
        let bounds = VariableBounds {
            min_position: -PI,
            max_position: PI,
            ..Default::default()
        };
        (joint, bounds)
    }

    #[test]
    fn set_axis_normalizes() {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 5.0));
        // normalize() divides an axis-aligned vector by its own exact norm
        // (5.0), giving (0.0, 0.0, 1.0) exactly; its norm is sqrt(1.0) = 1.0
        // exactly under IEEE 754 -- a structural identity, not a value
        // measured for this input alone.
        assert_eq!(joint.axis().norm(), 1.0);
    }

    /// `Eigen::Vector3d::normalized()` guards its zero-norm case and
    /// returns the input unchanged; `Vector3::normalize` has no such guard
    /// and turns a zero axis into `[NaN, NaN, NaN]`. A zero axis is
    /// reachable unscreened from URDF (`<axis xyz="0 0 0"/>`), so a NaN
    /// axis here would make `compute_transform` — and everything
    /// downstream of forward kinematics — NaN too. Fails before the
    /// `try_normalize` fix (axis is NaN, transform is NaN) and passes
    /// after.
    #[test]
    fn set_axis_on_a_zero_vector_leaves_it_zero_not_nan() {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::zeros());
        assert_eq!(joint.axis(), Vector3::zeros());
        let transform = joint.compute_transform(0.5);
        let q = transform.rotation.quaternion();
        assert!(q.i.is_finite() && q.j.is_finite() && q.k.is_finite() && q.w.is_finite());
    }

    /// Demonstrated opposite of the above: an ordinary non-unit axis still
    /// normalizes exactly, so the zero-norm guard does not turn `set_axis`
    /// into a no-op for every input.
    #[test]
    fn set_axis_still_normalizes_an_ordinary_non_unit_axis() {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 3.0));
        assert_eq!(joint.axis(), Vector3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn satisfies_position_bounds_at_and_outside_boundary_when_bounded() {
        let (joint, bounds) = bounded();
        assert!(joint.satisfies_position_bounds(1.0, &bounds, 0.0));
        assert!(!joint.satisfies_position_bounds(1.0 + f64::EPSILON * 4.0, &bounds, 0.0));
        assert!(joint.satisfies_position_bounds(1.0 + 0.5, &bounds, 0.5));
    }

    #[test]
    fn satisfies_position_bounds_ignores_bounds_when_continuous() {
        let (joint, bounds) = continuous();
        assert!(joint.satisfies_position_bounds(1000.0, &bounds, 0.0));
    }

    #[test]
    fn enforce_position_bounds_clamps_when_bounded() {
        let (joint, bounds) = bounded();
        let mut value = 5.0;
        assert!(joint.enforce_position_bounds(&mut value, &bounds));
        assert_eq!(value, 1.0);
    }

    #[test]
    fn enforce_position_bounds_wraps_when_continuous() {
        let (joint, bounds) = continuous();
        let mut value = PI + 0.5;
        joint.enforce_position_bounds(&mut value, &bounds);
        // `(PI + 0.5) - 2*PI + 0.5`-shaped wraparound measured exact for this
        // input; not asserted as a general property of `PI` arithmetic.
        assert_eq!(value, -PI + 0.5);
    }

    #[test]
    fn enforce_position_bounds_leaves_value_at_exactly_pi_when_continuous() {
        let (joint, bounds) = continuous();
        let mut value = PI;
        joint.enforce_position_bounds(&mut value, &bounds);
        assert_eq!(value, PI);
    }

    #[test]
    fn harmonize_position_wraps_regardless_of_continuous_flag() {
        let bounds = VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let mut value = -1.0 - 2.0 * PI;
        assert!(RevoluteJoint::harmonize_position(&mut value, &bounds));
        // Measured exact for this input; not asserted as a general property.
        assert_eq!(value, -1.0);
    }

    #[test]
    fn harmonize_position_is_noop_inside_bounds() {
        let bounds = VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let mut value = 0.5;
        assert!(!RevoluteJoint::harmonize_position(&mut value, &bounds));
        assert_eq!(value, 0.5);
    }

    #[test]
    fn interpolate_wraps_the_short_way_when_continuous() {
        let (joint, _bounds) = continuous();
        // From just past +pi to just past -pi the short way is forward through pi,
        // not backward across zero.
        let state = joint.interpolate(PI - 0.1, -PI + 0.1, 0.5);
        // Measured exact for this input; not asserted as a general property.
        assert_eq!(state.abs(), PI);
    }

    #[test]
    fn interpolate_is_linear_when_bounded() {
        let (joint, _bounds) = bounded();
        // Non-continuous branch is `from + (to - from) * t`; 0.0 + (1.0 -
        // 0.0) * 0.5 = 0.5 exactly under IEEE 754, not a value measured for
        // this input alone.
        assert_eq!(joint.interpolate(0.0, 1.0, 0.5), 0.5);
    }

    #[test]
    fn distance_takes_the_short_way_when_continuous() {
        let (joint, _bounds) = continuous();
        // The short-way distance goes through a modulo-based wrap, which
        // leaves a 1-ULP residue here (0.20000000000000018 vs 0.2) rather
        // than landing on the literal exactly.
        assert_relative_eq!(
            joint.distance(-PI + 0.1, PI - 0.1),
            0.2,
            epsilon = 1e-15,
            max_relative = 0.0
        );
    }

    #[test]
    fn distance_is_linear_when_bounded() {
        let (joint, _bounds) = bounded();
        // Non-continuous branch is `(value1 - value2).abs()`; (-1.0 -
        // 1.0).abs() = 2.0 exactly under IEEE 754, not a value measured for
        // this input alone.
        assert_eq!(joint.distance(-1.0, 1.0), 2.0);
    }

    #[test]
    fn compute_transform_round_trips_through_compute_variable_position() {
        let (joint, _bounds) = bounded();
        for value in [-0.75_f64, 0.0, 0.9] {
            let transform = joint.compute_transform(value);
            let recovered = joint.compute_variable_position(&transform);
            // Measured exact for these inputs; not asserted as a general
            // property of the round trip.
            assert_eq!(recovered, value);
        }
    }

    fn quaternion_coeffs(joint: &RevoluteJoint, value: f64) -> (f64, f64, f64, f64) {
        let transform = joint.compute_transform(value);
        let q = transform.rotation.quaternion();
        (q.i, q.j, q.k, q.w)
    }

    /// `value == 0.0` is where the degenerate-axis substitute is NOT a
    /// divergence: upstream's own `cos(0)*I` is exactly the identity,
    /// matching this port's identity substitute exactly. Covers the
    /// "never called `set_axis`" reachability path from
    /// `compute_transform`'s doc comment.
    #[test]
    fn compute_transform_on_a_never_set_axis_agrees_with_upstream_at_zero() {
        let joint = RevoluteJoint::default();
        assert_eq!(quaternion_coeffs(&joint, 0.0), (0.0, 0.0, 0.0, 1.0));
    }

    /// Same as above, through the other reachability path: `set_axis`
    /// called explicitly with a zero vector rather than never called.
    #[test]
    fn compute_transform_on_an_explicitly_zeroed_axis_agrees_with_upstream_at_zero() {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::zeros());
        assert_eq!(quaternion_coeffs(&joint, 0.0), (0.0, 0.0, 0.0, 1.0));
    }

    /// The disclosed divergence itself: for `value != 0`, upstream's own
    /// `cos(value)*I` is not a rotation at all (determinant
    /// `cos^3(value) != 1`), and this port's `Isometry3` cannot represent
    /// it -- see `compute_transform`'s doc comment. The identity
    /// substitute numerically disagrees with upstream's raw (invalid)
    /// output here, by design, not by bug.
    #[test]
    fn compute_transform_on_a_never_set_axis_returns_identity_not_upstreams_non_rotation() {
        let joint = RevoluteJoint::default();
        assert_eq!(quaternion_coeffs(&joint, 0.6), (0.0, 0.0, 0.0, 1.0));
    }

    /// Same disclosed divergence, through the other reachability path.
    #[test]
    fn compute_transform_on_an_explicitly_zeroed_axis_returns_identity_not_upstreams_non_rotation()
    {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::zeros());
        assert_eq!(quaternion_coeffs(&joint, 0.6), (0.0, 0.0, 0.0, 1.0));
    }

    /// Upstream treats "constructor, `setAxis` never called" and
    /// "`setAxis` called on a zero vector" as the identical `axis_ ==
    /// (0,0,0)` state (`revolute_joint_model.cpp:49` vs. `:75`); pinned
    /// here as literally equal transforms across several `value`s, not
    /// just individually-identity, so a future change to either
    /// reachability path that stops agreeing with the other is caught.
    #[test]
    fn compute_transform_treats_never_set_and_explicitly_zeroed_axis_as_the_same_state() {
        let never_set = RevoluteJoint::default();
        let mut explicitly_zeroed = RevoluteJoint::default();
        explicitly_zeroed.set_axis(Vector3::zeros());
        for value in [0.0_f64, 0.6, -1.2] {
            assert_eq!(
                quaternion_coeffs(&never_set, value),
                quaternion_coeffs(&explicitly_zeroed, value),
                "value = {value}"
            );
        }
    }

    /// Demonstrated opposite: an ordinary unit axis still produces the
    /// correct rotation -- the guard does not neuter `compute_transform`
    /// generally, only the degenerate-axis case.
    #[test]
    fn compute_transform_still_rotates_correctly_for_an_ordinary_axis() {
        let (joint, _bounds) = bounded(); // axis (0, 0, 1)
        let (i, j, k, w) = quaternion_coeffs(&joint, 0.6);
        // Rotation of 0.6 rad about +z: (0, 0, sin(0.3), cos(0.3)) exactly,
        // under IEEE 754, not a value measured for this input alone.
        assert_eq!((i, j), (0.0, 0.0));
        assert_eq!(k, 0.3_f64.sin());
        assert_eq!(w, 0.3_f64.cos());
    }

    /// `set_axis` on a NaN input propagates the NaN (unlike the zero-axis
    /// case, which `try_normalize` guards -- see `set_axis`'s doc comment),
    /// so a NaN axis is reachable here. `partial_cmp(...).unwrap()` (the
    /// old `max_by`-based spelling) panics outright when comparing any NaN
    /// component; `reduce`'s explicit `>` comparison does not, matching
    /// Eigen's measured `maxCoeff` rule of never displacing the incumbent
    /// on a NaN challenger. Written as a normal call, not `#[should_panic]`,
    /// so it fails before this fix (the old code panics) and passes after.
    #[test]
    fn compute_variable_position_on_a_nan_axis_returns_rather_than_panics() {
        let joint = RevoluteJoint {
            axis: Vector3::new(f64::NAN, 1.0, 0.0),
            continuous: false,
        };
        let transform = Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            UnitQuaternion::identity(),
        );
        // Eigen's fold never displaces the incumbent on a NaN challenger, so
        // the leading NaN component (index 0) is kept, and the returned
        // angle is NaN, not a panic.
        assert!(joint.compute_variable_position(&transform).is_nan());
    }

    /// Demonstrated opposite of the above: an ordinary unit axis (no NaN
    /// component) still recovers the same angle after the `reduce` rewrite
    /// that `max_by` gave before it.
    #[test]
    fn compute_variable_position_still_recovers_the_angle_for_an_ordinary_axis() {
        let (joint, _bounds) = bounded(); // axis (0, 0, 1), no tie possible
        let transform = joint.compute_transform(0.9);
        assert_eq!(joint.compute_variable_position(&transform), 0.9);
    }
}
