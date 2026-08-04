// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/planar_joint_model.hpp
//   moveit_core/robot_model/src/planar_joint_model.cpp

use std::f64::consts::PI;

use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};

use super::bounds::VariableBounds;

/// A planar joint: 3 degrees of freedom, `x`, `y` translation in a plane
/// plus `theta` rotation about its normal.
///
/// Upstream `moveit::core::PlanarJointModel`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarJoint {
    angular_distance_weight: f64,
    motion_model: PlanarMotionModel,
    /// Only used by [`PlanarMotionModel::DiffDrive`]: below this
    /// translational distance, `interpolate`/`distance` treat the motion as
    /// pure rotation rather than turn-drive-turn, to avoid an unnecessary
    /// double turn when `from` and `to` are almost the same point (see
    /// upstream's `computeTurnDriveTurnGeometry` comment).
    min_translational_distance: f64,
}

/// Upstream `PlanarJointModel::MotionModel`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlanarMotionModel {
    /// Free motion in the plane. Upstream's default.
    #[default]
    Holonomic,
    /// Turn-drive-turn motion, as for a differential-drive base.
    DiffDrive,
}

impl Default for PlanarJoint {
    fn default() -> Self {
        Self {
            angular_distance_weight: 1.0,
            motion_model: PlanarMotionModel::Holonomic,
            min_translational_distance: 1e-5,
        }
    }
}

impl PlanarJoint {
    /// The weight applied to the rotational component of `PlanarJoint::distance`
    /// relative to the translational component (which has weight 1).
    pub fn angular_distance_weight(&self) -> f64 {
        self.angular_distance_weight
    }

    /// Set [`PlanarJoint::angular_distance_weight`].
    pub fn set_angular_distance_weight(&mut self, weight: f64) {
        self.angular_distance_weight = weight;
    }

    /// See this type's `min_translational_distance` field doc comment.
    pub fn min_translational_distance(&self) -> f64 {
        self.min_translational_distance
    }

    /// Set [`PlanarJoint::min_translational_distance`].
    pub fn set_min_translational_distance(&mut self, distance: f64) {
        self.min_translational_distance = distance;
    }

    /// Holonomic or differential-drive motion.
    pub fn motion_model(&self) -> PlanarMotionModel {
        self.motion_model
    }

    /// Set [`PlanarJoint::motion_model`].
    pub fn set_motion_model(&mut self, model: PlanarMotionModel) {
        self.motion_model = model;
    }

    pub(super) fn default_position(bounds: &[VariableBounds; 3]) -> [f64; 3] {
        let mut values = [0.0; 3];
        for i in 0..2 {
            values[i] = if bounds[i].min_position <= 0.0 && bounds[i].max_position >= 0.0 {
                0.0
            } else {
                (bounds[i].min_position + bounds[i].max_position) / 2.0
            };
        }
        values
    }

    pub(super) fn maximum_extent(&self, bounds: &[VariableBounds; 3]) -> f64 {
        let dx = bounds[0].max_position - bounds[0].min_position;
        let dy = bounds[1].max_position - bounds[1].min_position;
        (dx * dx + dy * dy).sqrt() + PI * self.angular_distance_weight
    }

    pub(super) fn satisfies_position_bounds(
        values: &[f64; 3],
        bounds: &[VariableBounds; 3],
        margin: f64,
    ) -> bool {
        (0..3).all(|i| {
            values[i] >= bounds[i].min_position - margin
                && values[i] <= bounds[i].max_position + margin
        })
    }

    /// Bring `values[2]` (theta) into `[-pi, pi]`. Upstream
    /// `PlanarJointModel::normalizeRotation`.
    pub fn normalize_rotation(values: &mut [f64; 3]) -> bool {
        let v = &mut values[2];
        if *v >= -PI && *v <= PI {
            return false;
        }
        *v %= 2.0 * PI;
        if *v < -PI {
            *v += 2.0 * PI;
        } else if *v > PI {
            *v -= 2.0 * PI;
        }
        true
    }

    pub(super) fn enforce_position_bounds(
        values: &mut [f64; 3],
        bounds: &[VariableBounds; 3],
    ) -> bool {
        let mut result = Self::normalize_rotation(values);
        for i in 0..2 {
            if values[i] < bounds[i].min_position {
                values[i] = bounds[i].min_position;
                result = true;
            } else if values[i] > bounds[i].max_position {
                values[i] = bounds[i].max_position;
                result = true;
            }
        }
        result
    }

    pub(super) fn distance(&self, values1: &[f64; 3], values2: &[f64; 3]) -> f64 {
        match self.motion_model {
            PlanarMotionModel::Holonomic => {
                let dx = values1[0] - values2[0];
                let dy = values1[1] - values2[1];
                let d = (values1[2] - values2[2]).abs();
                let d = if d > PI { 2.0 * PI - d } else { d };
                (dx * dx + dy * dy).sqrt() + self.angular_distance_weight * d
            }
            PlanarMotionModel::DiffDrive => {
                let g = turn_drive_turn_geometry(values1, values2, self.min_translational_distance);
                g.dx.hypot(g.dy)
                    + self.angular_distance_weight * (g.initial_turn.abs() + g.final_turn.abs())
            }
        }
    }

    pub(super) fn interpolate(&self, from: &[f64; 3], to: &[f64; 3], t: f64) -> [f64; 3] {
        match self.motion_model {
            PlanarMotionModel::Holonomic => {
                let x = from[0] + (to[0] - from[0]) * t;
                let y = from[1] + (to[1] - from[1]) * t;
                let diff = to[2] - from[2];
                let theta = if diff.abs() <= PI {
                    from[2] + diff * t
                } else {
                    let diff = if diff > 0.0 {
                        2.0 * PI - diff
                    } else {
                        -2.0 * PI - diff
                    };
                    let mut theta = from[2] - diff * t;
                    if theta > PI {
                        theta -= 2.0 * PI;
                    } else if theta < -PI {
                        theta += 2.0 * PI;
                    }
                    theta
                };
                [x, y, theta]
            }
            PlanarMotionModel::DiffDrive => {
                let g = turn_drive_turn_geometry(from, to, self.min_translational_distance);
                let initial_d = g.initial_turn.abs() * self.angular_distance_weight;
                let drive_d = g.dx.hypot(g.dy);
                let final_d = g.final_turn.abs() * self.angular_distance_weight;
                let total_d = initial_d + drive_d + final_d;
                let initial_frac = initial_d / total_d;
                let drive_frac = drive_d / total_d;

                if t <= initial_frac {
                    let percent = t / initial_frac;
                    [from[0], from[1], from[2] + g.initial_turn * percent]
                } else if t <= initial_frac + drive_frac {
                    let percent = (t - initial_frac) / drive_frac;
                    [
                        from[0] + g.dx * percent,
                        from[1] + g.dy * percent,
                        g.drive_angle,
                    ]
                } else {
                    let final_frac = final_d / total_d;
                    let percent = (t - initial_frac - drive_frac) / final_frac;
                    [to[0], to[1], g.drive_angle + g.final_turn * percent]
                }
            }
        }
    }

    pub(super) fn compute_transform(values: &[f64; 3]) -> Isometry3 {
        Isometry3::from_parts(
            nalgebra::Translation3::new(values[0], values[1], 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), values[2]),
        )
    }

    pub(super) fn compute_variable_positions(transform: &Isometry3) -> [f64; 3] {
        let translation = transform.translation.vector;
        let q = transform.rotation.quaternion();
        let (mut w, mut k) = (q.w, q.k);
        if w < 0.0 {
            w = -w;
            k = -k;
        }
        let s_squared = 1.0 - w * w;
        let theta = if s_squared < 10.0 * f64::EPSILON {
            0.0
        } else {
            let s = 1.0 / s_squared.sqrt();
            (w.acos() * 2.0) * (k * s)
        };
        [translation.x, translation.y, theta]
    }
}

struct TurnDriveTurnGeometry {
    dx: f64,
    dy: f64,
    initial_turn: f64,
    drive_angle: f64,
    final_turn: f64,
}

/// Upstream `moveit::core::computeTurnDriveTurnGeometry`: the geometry for a
/// differential-drive base to turn toward `to`, drive straight, then turn to
/// `to`'s final orientation.
fn turn_drive_turn_geometry(
    from: &[f64; 3],
    to: &[f64; 3],
    min_translational_distance: f64,
) -> TurnDriveTurnGeometry {
    let dx = to[0] - from[0];
    let dy = to[1] - from[1];
    let angle_straight_diff = if dx.hypot(dy) > min_translational_distance {
        shortest_angular_distance(from[2], dy.atan2(dx))
    } else {
        0.0
    };
    let angle_backward_diff = normalize_angle(angle_straight_diff - PI);
    let move_straight_cost = angle_straight_diff.abs()
        + shortest_angular_distance(from[2] + angle_straight_diff, to[2]).abs();
    let move_backward_cost = angle_backward_diff.abs()
        + shortest_angular_distance(from[2] + angle_backward_diff, to[2]).abs();
    let initial_turn = if move_straight_cost <= move_backward_cost {
        angle_straight_diff
    } else {
        angle_backward_diff
    };
    let drive_angle = from[2] + initial_turn;
    let final_turn = shortest_angular_distance(drive_angle, to[2]);
    TurnDriveTurnGeometry {
        dx,
        dy,
        initial_turn,
        drive_angle,
        final_turn,
    }
}

// Ported from ROS package `angles` @ 1.16.1:
//   include/angles/angles/angles.h (normalize_angle, shortest_angular_distance)

/// Bring `angle` into `(-pi, pi]`, matching the ROS `angles` package's
/// `normalize_angle`.
///
/// Only exercised by the diff-drive motion model, which neither the panda
/// nor the fanuc fixture uses.
fn normalize_angle(angle: f64) -> f64 {
    let result = (angle + PI) % (2.0 * PI);
    if result <= 0.0 {
        result + PI
    } else {
        result - PI
    }
}

/// Upstream `angles::shortest_angular_distance`.
fn shortest_angular_distance(from: f64, to: f64) -> f64 {
    normalize_angle(to - from)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    fn xy_bounds() -> [VariableBounds; 3] {
        [
            VariableBounds {
                min_position: -1.0,
                max_position: 1.0,
                position_bounded: true,
                ..Default::default()
            },
            VariableBounds {
                min_position: -1.0,
                max_position: 1.0,
                position_bounded: true,
                ..Default::default()
            },
            VariableBounds {
                min_position: -PI,
                max_position: PI,
                ..Default::default()
            },
        ]
    }

    #[test]
    fn normalize_angle_returns_positive_pi_at_the_pi_boundary() {
        // angles::normalize_angle's range is (-pi, pi] — the upper bound is
        // closed, the lower bound is open — so angle == pi (mod 2*pi) must
        // come back as +pi, never -pi.
        assert_eq!(normalize_angle(PI), PI);
        assert_eq!(normalize_angle(-PI), PI);
        assert_eq!(normalize_angle(3.0 * PI), PI);
    }

    #[test]
    fn normalize_angle_wraps_just_past_pi_to_just_past_negative_pi() {
        // `normalize_angle`'s wrap subtracts a multiple of `2*PI`, which
        // leaves a few-ULP residue near the pi boundary rather than landing
        // on the literal exactly.
        assert_relative_eq!(
            normalize_angle(PI + 0.1),
            -PI + 0.1,
            epsilon = 1e-15,
            max_relative = 0.0
        );
    }

    #[test]
    fn shortest_angular_distance_is_zero_for_equal_angles() {
        assert_eq!(shortest_angular_distance(1.23, 1.23), 0.0);
    }

    #[test]
    fn normalize_rotation_is_noop_at_exactly_pi() {
        let mut values = [0.0, 0.0, PI];
        assert!(!PlanarJoint::normalize_rotation(&mut values));
        assert_eq!(values[2], PI);
    }

    #[test]
    fn normalize_rotation_wraps_just_past_pi() {
        let mut values = [0.0, 0.0, PI + 0.1];
        assert!(PlanarJoint::normalize_rotation(&mut values));
        // Measured exact for this input; not asserted as a general property.
        assert_eq!(values[2], -PI + 0.1);
    }

    #[test]
    fn satisfies_position_bounds_at_and_outside_boundary() {
        let bounds = xy_bounds();
        assert!(PlanarJoint::satisfies_position_bounds(
            &[1.0, 1.0, 0.0],
            &bounds,
            0.0
        ));
        assert!(!PlanarJoint::satisfies_position_bounds(
            &[1.0 + f64::EPSILON * 4.0, 0.0, 0.0],
            &bounds,
            0.0
        ));
    }

    #[test]
    fn enforce_position_bounds_clamps_translation_and_normalizes_rotation() {
        let bounds = xy_bounds();
        let mut values = [5.0, -5.0, PI + 0.1];
        assert!(PlanarJoint::enforce_position_bounds(&mut values, &bounds));
        assert_eq!(values[0], 1.0);
        assert_eq!(values[1], -1.0);
        // Measured exact for this input; not asserted as a general property.
        assert_eq!(values[2], -PI + 0.1);
    }

    #[test]
    fn holonomic_distance_takes_the_short_way_around_theta() {
        let joint = PlanarJoint {
            motion_model: PlanarMotionModel::Holonomic,
            ..Default::default()
        };
        let d = joint.distance(&[0.0, 0.0, -PI + 0.1], &[0.0, 0.0, PI - 0.1]);
        // The short-way distance goes through a modulo-based wrap, which
        // leaves a 1-ULP residue here (0.20000000000000018 vs 0.2) rather
        // than landing on the literal exactly -- the same pattern as
        // `RevoluteJoint::distance`'s equivalent test.
        assert_relative_eq!(d, 0.2, epsilon = 1e-15, max_relative = 0.0);
    }

    #[test]
    fn holonomic_interpolate_wraps_the_short_way_when_diff_exceeds_pi() {
        let joint = PlanarJoint {
            motion_model: PlanarMotionModel::Holonomic,
            ..Default::default()
        };
        let state = joint.interpolate(&[0.0, 0.0, PI - 0.1], &[0.0, 0.0, -PI + 0.1], 0.5);
        // Measured exact for this input; not asserted as a general property.
        assert_eq!(state[2].abs(), PI);
    }

    #[test]
    fn holonomic_interpolate_is_linear_when_diff_is_exactly_pi() {
        let joint = PlanarJoint {
            motion_model: PlanarMotionModel::Holonomic,
            ..Default::default()
        };
        let state = joint.interpolate(&[0.0, 0.0, 0.0], &[0.0, 0.0, PI], 0.5);
        // `0.0 + (PI - 0.0) * 0.5` is exact under IEEE 754.
        assert_eq!(state[2], PI / 2.0);
    }

    #[test]
    fn diff_drive_below_min_translational_distance_skips_the_initial_turn() {
        let joint = PlanarJoint {
            motion_model: PlanarMotionModel::DiffDrive,
            min_translational_distance: 1e-3,
            ..Default::default()
        };
        // from and to differ only in theta, well below min_translational_distance apart.
        let g = turn_drive_turn_geometry(
            &[0.0, 0.0, 0.0],
            &[1e-6, 0.0, PI / 2.0],
            joint.min_translational_distance,
        );
        assert_eq!(g.initial_turn, 0.0);
    }

    #[test]
    fn compute_transform_round_trips_through_compute_variable_positions() {
        for theta in [-2.5_f64, 0.0, 1.2] {
            let transform = PlanarJoint::compute_transform(&[0.3, -0.7, theta]);
            let recovered = PlanarJoint::compute_variable_positions(&transform);
            // Measured exact for these inputs; not asserted as a general
            // property of the round trip.
            assert_eq!(recovered[0], 0.3);
            assert_eq!(recovered[1], -0.7);
            assert_eq!(recovered[2], theta);
        }
    }

    #[test]
    fn compute_variable_positions_canonicalizes_negative_w() {
        // theta = 4.0 exceeds pi, so half_angle = 2.0 exceeds pi/2 and the
        // underlying quaternion's w = cos(half_angle) goes negative.
        // compute_variable_positions must still recover a theta that is the
        // *same rotation* (it wraps into (-pi, pi], not necessarily 4.0
        // itself), via the w<0 sign flip.
        let theta = 4.0_f64;
        let transform = PlanarJoint::compute_transform(&[0.0, 0.0, theta]);
        assert!(transform.rotation.quaternion().w < 0.0);
        let recovered = PlanarJoint::compute_variable_positions(&transform);
        let round_tripped = PlanarJoint::compute_transform(&[0.0, 0.0, recovered[2]]);
        // `angle_to` goes through a quaternion round trip (build, negate on
        // w<0, recover, rebuild), which leaves a few-ULP residue rather than
        // landing on 0.0 exactly.
        assert_relative_eq!(
            round_tripped.rotation.angle_to(&transform.rotation),
            0.0,
            epsilon = 1e-15,
            max_relative = 0.0
        );
    }
}
