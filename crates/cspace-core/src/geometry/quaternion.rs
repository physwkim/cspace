// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Transcribed from Eigen 3.4, Eigen/src/Geometry/Quaternion.h:782
// (`QuaternionBase<Derived>::slerp`), read out of the oracle image the
// pinned moveit2 tree is built in.

//! `Eigen::Quaterniond::slerp`, transcribed.
//!
//! This is not a port of any moveit2 file. It exists because three ported
//! call sites reach `Eigen::QuaternionBase::slerp` through upstream code
//! ([`FloatingJointModel::interpolate`][fj], `CartesianInterpolator`,
//! `TrajectoryBlenderTransitionWindow`), and the obvious `nalgebra`
//! substitute, `UnitQuaternion::try_slerp`, **is a different function**.
//! The differences were measured against the real C++ oracle, not reasoned
//! about; see [`slerp_coefficients`] for the three of them and their
//! magnitudes.
//!
//! [fj]: https://github.com/moveit/moveit2/blob/e017c91ee12984393a28ba246075c65f69cde3bf/moveit_core/robot_model/src/floating_joint_model.cpp
//!
//! # Why one shared transcription rather than three local ones
//!
//! The first version of this lived privately in `cspace-model`'s
//! `floating.rs`. Sweeping the anchor `rg -n 'slerp' crates/` found two more
//! sites porting the same Eigen call — `cspace-kinematics`'
//! `cartesian_interpolator.rs` and `cspace-planners-pilz`'
//! `trajectory_blender_transition_window.rs` — and one that is **not** the
//! same defect: `cspace-planners-sbp`'s `se3.rs`, which hand-writes OMPL's
//! own slerp because it ports OMPL's `SO3StateSpace::interpolate`, not
//! Eigen's. A single definition here is what keeps the first three from
//! drifting apart again.

use nalgebra::Quaternion;

use crate::geometry::UnitQuaternion;

/// `Eigen::QuaternionBase::slerp` (Eigen 3.4,
/// `Eigen/src/Geometry/Quaternion.h:782`), on raw `xyzw` coefficients —
/// which is Eigen's own `coeffs()` order, so this is the same expression
/// component for component.
///
/// Transcribed rather than delegated to `nalgebra`'s
/// `UnitQuaternion::try_slerp`. The two are not the same function, and a
/// live comparison against the C++ oracle found all three differences on
/// `fixtures/panda.urdf`'s floating virtual joint:
///
/// 1. **The near-parallel branch is entered at a different point and does a
///    different thing.** Eigen takes it at `|d| >= 1 - ε` and *lerps*
///    (`scale0 = 1 - t`, `scale1 = t`); nalgebra takes it at `|d| >= 1` and
///    returns `from` unchanged. Two quaternions a few ULP apart therefore
///    diverged by up to `8.9e-16` at `t = 1`.
/// 2. **Unnormalized input is not a degenerate case for Eigen.** `d` is the
///    raw dot product, so any same-direction pair with norm above 1 has
///    `|d| >= 1 - ε` and lerps; nalgebra's `|c_hang| >= 1` returned `from`
///    for *every* `t`, so interpolating between two norm-2 quaternions never
///    moved at all — measured `1.414` off at `t = 1`. `enforcePositionBounds`
///    is what normalizes a stored quaternion, and nothing requires it to have
///    run before an interpolation does.
/// 3. **Eigen does not normalize the result; nalgebra does**
///    (`res.normalize_mut()` inside `Unit::try_slerp`). On a `from` whose
///    norm is 1 only to within a ULP, that rewrote the `t = 0` answer, which
///    upstream returns bit-for-bit — measured `1.25e-13` off.
///
/// # Totality
///
/// This expression cannot panic and cannot produce a division by zero, so
/// the `nlerp` fallbacks that used to guard `try_slerp`'s "ambiguous
/// configuration" `None` are gone with the call that could return it.
/// `d.abs() >= 1 - ε` covers the exactly-antipodal case (`d = -1`) through
/// the lerp branch, where `scale0 = 1 - t` and `scale1 = -t` reconstruct
/// `from` for every `t` — Eigen's answer, and not `nlerp`'s, which
/// degenerates to `0/0` at `t = 0.5`. `sin_theta` is only ever divided by
/// under `|d| < 1 - ε`, where `theta = acos(|d|) > sqrt(2ε)` bounds it below.
pub fn slerp_coefficients(from: &[f64; 4], to: &[f64; 4], t: f64) -> [f64; 4] {
    let one = 1.0 - f64::EPSILON;
    let d: f64 = (0..4).map(|i| from[i] * to[i]).sum();
    let (scale0, mut scale1) = if d.abs() >= one {
        (1.0 - t, t)
    } else {
        let theta = d.abs().acos();
        let sin_theta = theta.sin();
        (
            ((1.0 - t) * theta).sin() / sin_theta,
            (t * theta).sin() / sin_theta,
        )
    };
    if d < 0.0 {
        scale1 = -scale1;
    }
    let mut out = [0.0; 4];
    for i in 0..4 {
        out[i] = scale0 * from[i] + scale1 * to[i];
    }
    out
}

/// [`slerp_coefficients`] on [`UnitQuaternion`]s.
///
/// The result is built with `UnitQuaternion::new_unchecked` on purpose:
/// Eigen returns the raw combination and every upstream consumer of it
/// (`Eigen::Isometry3d`'s quaternion constructor, `toRotationMatrix()`)
/// consumes it without normalizing, so normalizing here would be a
/// divergence rather than a repair. For unit inputs the deviation from unit
/// norm is bounded by the rounding of the two-term combination — the
/// near-parallel branch lerps across an arc of at most `sqrt(2ε)` radians,
/// costing `θ²/8 < 6e-17` of norm, and the `sin` branch is the exact slerp
/// identity. Non-unit *inputs* are the caller's own business, and difference
/// 2 above is precisely the case where preserving them matters.
pub fn slerp(from: &UnitQuaternion, to: &UnitQuaternion, t: f64) -> UnitQuaternion {
    let f = from.as_ref().coords;
    let g = to.as_ref().coords;
    let out = slerp_coefficients(&[f.x, f.y, f.z, f.w], &[g.x, g.y, g.z, g.w], t);
    UnitQuaternion::new_unchecked(Quaternion::new(out[3], out[0], out[1], out[2]))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `d.abs() >= one` with `d > 0`: Eigen lerps, and the lerp of two
    /// quaternions a few ULP apart is *not* `from`. This is the boundary
    /// nalgebra puts at `|d| >= 1` instead, so a same-side-of-1 pair takes
    /// opposite branches in the two libraries.
    #[test]
    fn near_parallel_branch_lerps_rather_than_copying_from() {
        let from = [0.0, 0.0, 0.0, 1.0];
        let to = [0.0, 0.0, 0.0, 1.0 + 8.0 * f64::EPSILON];
        let got = slerp_coefficients(&from, &to, 1.0);
        assert!(
            (got[3] - 1.0).abs() > 0.0,
            "lerp at t=1 must reach `to`, not stay at `from`; got {got:?}"
        );
        assert_eq!(got[3], 1.0 + 8.0 * f64::EPSILON);
    }

    /// The threshold itself, `d == 1 - EPSILON`, which is the only `d` at
    /// which Eigen's `1 - EPSILON` and nalgebra's `1` disagree: measured over
    /// `t` in 100 steps, the two branches are bit-identical at `1 - 2^-53`
    /// and `1 - 3*2^-53` and differ by `2.22e-16` here. Without this case the
    /// constant is unpinned -- `near_parallel_branch_*` above sits at `d > 1`,
    /// where both thresholds lerp.
    ///
    /// `t` must be interior. Both branches land on `from` at `t = 0` and on
    /// `to` at `t = 1` (`sin(theta)/sin(theta)` is exactly `1.0`), so an
    /// endpoint case cannot see which one ran.
    #[test]
    fn the_threshold_value_itself_takes_the_lerp_branch() {
        let w = 1.0 - f64::EPSILON;
        let from = [0.0, 0.0, 0.0, 1.0];
        let to = [(1.0f64 - w * w).sqrt(), 0.0, 0.0, w];
        let t = 1e-9;
        let mut lerped = [0.0; 4];
        for i in 0..4 {
            lerped[i] = (1.0 - t) * from[i] + t * to[i];
        }
        assert_eq!(slerp_coefficients(&from, &to, t), lerped);
    }

    /// `d.abs() >= one` with `d < 0`: `scale1` is negated, and for an exactly
    /// antipodal pair that reconstructs `from` at every `t`. The value this
    /// pins is the *absence* of a `0/0`: `nlerp` on the same input divides by
    /// a zero-norm sum at `t = 0.5`.
    #[test]
    fn antipodal_pair_returns_from_at_every_t() {
        let from = [0.0, 0.0, 0.0, 1.0];
        let to = [-0.0, -0.0, -0.0, -1.0];
        for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let got = slerp_coefficients(&from, &to, t);
            assert_eq!(got, from, "antipodal slerp at t={t} must be `from`");
        }
    }

    /// `d.abs() < one`: the `sin`-quotient branch, checked against the
    /// closed-form half-angle it must reproduce. A 90° pair at `t = 0.5` is
    /// 45°, whose coefficients are `cos/sin(22.5°)`.
    #[test]
    fn sin_branch_reproduces_the_half_angle() {
        let half = std::f64::consts::FRAC_PI_4;
        let from = [0.0, 0.0, 0.0, 1.0];
        let to = [0.0, 0.0, half.sin(), half.cos()];
        let got = slerp_coefficients(&from, &to, 0.5);
        let quarter = std::f64::consts::FRAC_PI_8;
        assert!((got[2] - quarter.sin()).abs() < 1e-15, "got {got:?}");
        assert!((got[3] - quarter.cos()).abs() < 1e-15, "got {got:?}");
    }

    /// Eigen normalizes neither its inputs nor its result. A norm-2 pair must
    /// therefore still move with `t` (nalgebra returned `from` for every `t`
    /// here) and must stay at norm 2 rather than being renormalized to 1.
    #[test]
    fn unnormalized_input_neither_freezes_nor_renormalizes() {
        let from = [0.0, 0.0, 0.0, 2.0];
        let to = [
            0.0,
            0.0,
            2.0 * std::f64::consts::FRAC_1_SQRT_2,
            2.0 * std::f64::consts::FRAC_1_SQRT_2,
        ];
        let got = slerp_coefficients(&from, &to, 1.0);
        assert_eq!(got, to, "t=1 on the lerp branch must land exactly on `to`");
        let norm: f64 = got.iter().map(|c| c * c).sum::<f64>().sqrt();
        assert!((norm - 2.0).abs() < 1e-15, "norm was renormalized: {norm}");
    }

    /// The `sin` branch at `t = 0` must return `from` bit-for-bit, including
    /// when `from`'s own norm is off by a ULP. Normalizing the result is what
    /// broke this upstream-exact identity by `1.25e-13` on the oracle sweep.
    #[test]
    fn t_zero_returns_from_bit_for_bit() {
        let from = [0.0, 0.0, 0.7071067811864225, 0.7071067811864225];
        let to = [0.0, 0.0, -0.7071067811865475, 0.7071067811865475];
        let got = slerp_coefficients(&from, &to, 0.0);
        assert_eq!(got, from);
    }

    /// The [`UnitQuaternion`] wrapper must reorder `xyzw` correctly in both
    /// directions; a `wxyz`/`xyzw` swap is invisible on any case whose
    /// rotation is about a single axis with equal components.
    #[test]
    fn unit_wrapper_preserves_component_order() {
        let from = UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        let axis = nalgebra::Vector3::x_axis();
        let to = UnitQuaternion::from_axis_angle(&axis, std::f64::consts::FRAC_PI_2);
        let got = slerp(&from, &to, 1.0);
        assert_eq!(got.as_ref().coords, to.as_ref().coords);
    }
}
