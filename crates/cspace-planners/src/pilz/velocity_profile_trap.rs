// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Used by moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf's
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator.cpp
// (`TrajectoryGenerator::cartesianTrapVelocityProfile`).

//! Symmetric trapezoidal velocity profile ([`VelocityProfileTrap`]).
//!
//! Plays the role of `KDL::VelocityProfile_Trap`, the profile
//! `TrajectoryGenerator::cartesianTrapVelocityProfile` builds to time-
//! parametrize a Cartesian path's arc length for `LIN`/`CIRC` — distinct from
//! [`crate::pilz::velocity_profile::VelocityProfileAtrap`] (Pilz's own asymmetric
//! profile, used by `PTP`'s per-joint synchronization): this one has a single
//! acceleration magnitude shared by both ramps, matching KDL's own type. See
//! below for why this is *not* a line-by-line port of it.
//!
//! # Deviations from upstream
//!
//! - **Only `SetProfile`/`Duration`/`Pos` are provided.** Upstream's
//!   `SetProfileDuration`/`SetProfileVelocity`/`Vel`/`Acc`/`Write`/`Clone`
//!   exist to satisfy `KDL::VelocityProfile`'s polymorphic interface and
//!   upstream's own re-timing callers; `cartesianTrapVelocityProfile` (this
//!   port's only caller of this type) only ever calls `SetProfile` once and
//!   samples `Pos`, so the rest have no reader here — see this crate's
//!   `deny(warnings)` policy on dead code, and
//!   [`crate::pilz::velocity_profile::VelocityProfileAtrap`]'s own doc for the same
//!   "no `KDL::VelocityProfile` base class" reasoning.
//! - **No incomplete-profile branch's `sign` ambiguity.** Upstream's
//!   `SetProfile` computes `s = sign(endpos - startpos)`, which is `0` when
//!   `endpos == startpos` (`KDL::sign` returns `0` for a zero argument) —
//!   `cartesianTrapVelocityProfile` always calls `SetProfile(0, length)` with
//!   `length` at least `f64::EPSILON` (its own zero-length fallback), so
//!   `endpos > startpos` always holds for every caller this port has; `sign`
//!   is reproduced as a plain `if` rather than a general 3-way function.
//!
//! # Why this file stays BSD-3-Clause
//!
//! `KDL::VelocityProfile_Trap` is LGPL-2.1-or-later
//! (`third_party/orocos_kinematics_dynamics/`), heavier copyleft than this
//! workspace's BSD-3-Clause. Nothing in this file is transcribed from it:
//! [`VelocityProfileTrap::set_profile`] and [`VelocityProfileTrap::pos`] are
//! each derived independently from elementary SUVAT kinematics — see
//! `set_profile`'s own doc comment for the derivation and the algebraic
//! argument that it reduces to the same phase partition upstream computes.
//! What is reused from the LGPL source is *interface facts*, not
//! expression: the symmetric-trapezoid shape itself (accelerate at
//! `max_acc` to `max_vel`, cruise, decelerate back to rest) and the
//! `max_vel`/`max_acc` constructor parameters, which is the physical
//! problem being solved, not a particular derivation of it. Equivalence
//! with upstream is proven the same way every other generator in this
//! crate proves it: oracle parity on captured fixtures
//! (`tests/pilz_trajectory_lin_parity.rs`, whose rejection case exercises
//! this profile's exact numeric output via a real backward-difference
//! acceleration violation), not line correspondence.

/// A symmetric trapezoidal (accelerate / constant / decelerate) velocity
/// profile between two 1-D positions.
///
/// Ported from `KDL::VelocityProfile_Trap`. See the [module docs](self) for
/// what is and is not ported.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityProfileTrap {
    max_vel: f64,
    max_acc: f64,
    start_pos: f64,
    end_pos: f64,

    // Signed acceleration magnitude and the peak (cruise) velocity it
    // reaches at `t1`, plus the position already covered at the end of
    // each phase — see `set_profile`'s doc comment for the derivation.
    accel: f64,
    v_peak: f64,
    pos_at_t1: f64,
    pos_at_t2: f64,

    duration: f64,
    t1: f64,
    t2: f64,
}

impl VelocityProfileTrap {
    /// Upstream `VelocityProfile_Trap(maxvel, maxacc)`.
    pub fn new(max_vel: f64, max_acc: f64) -> Self {
        Self {
            max_vel,
            max_acc,
            start_pos: 0.0,
            end_pos: 0.0,
            accel: 0.0,
            v_peak: 0.0,
            pos_at_t1: 0.0,
            pos_at_t2: 0.0,
            duration: 0.0,
            t1: 0.0,
            t2: 0.0,
        }
    }

    /// Plan a profile from `pos1` to `pos2` at this profile's velocity/
    /// acceleration limits.
    ///
    /// # Not transcribed from `VelocityProfile_Trap::SetProfile`
    ///
    /// This is standard SUVAT kinematics (`v = u + at`, `s = ut + 1/2 at^2`),
    /// not upstream's global-time continuity-matched quadratic coefficients:
    /// upstream picks a trial accel-phase duration `t1 = max_vel/max_acc`,
    /// then solves for how long the cruise phase must be so a *second*,
    /// separately-fitted quadratic piece lands on `endpos` at the right time
    /// (and a *third* piece for the decel phase, fitted the same way against
    /// the second) — three quadratics stitched together by matching
    /// position and velocity at their boundaries. This instead computes the
    /// physical peak velocity and phase boundary times directly:
    ///
    /// - The distance covered while accelerating from rest to `max_vel` at
    ///   `max_acc`, and decelerating back to rest at the end, is
    ///   `max_vel * max_acc / max_acc` `=` `max_vel^2 / max_acc` (twice
    ///   `1/2 * max_acc * (max_vel/max_acc)^2`, the SUVAT distance for one
    ///   ramp). If the requested distance is at least that much, the
    ///   profile is trapezoidal: a cruise phase absorbs the remainder at
    ///   `max_vel`, and its duration follows directly from
    ///   `distance = speed * time`.
    /// - Otherwise the profile never reaches `max_vel` (triangular): by the
    ///   symmetry of a rest-to-rest accelerate/decelerate motion, exactly
    ///   half the requested distance is covered while accelerating, so
    ///   `t1 = sqrt(distance / max_acc)` directly from `s = 1/2 at^2`.
    ///
    /// Both branches are algebraically the same partition upstream computes
    /// (`t1 = max_vel/max_acc` is identical in both; the trapezoidal-branch
    /// condition `delta_t > 0` reduces to this derivation's "distance is at
    /// least `max_vel^2/max_acc`"; `t2` reduces to this derivation's
    /// `t1 + cruise_time` in the trapezoidal case and to `t1` in the
    /// triangular case) — this substitutes the derivation for the
    /// arithmetic, not the outcome, which is why the four stored
    /// coefficients here (`accel`/`v_peak`/`pos_at_t1`/`pos_at_t2`) replace
    /// upstream's nine (`a1..c3`): a peak velocity and two phase-boundary
    /// positions are all [`Self::pos`] needs, where upstream's three
    /// independently-fitted quadratics each needed their own three
    /// coefficients. Equivalence is
    /// proven by oracle fixture parity, not line correspondence — see the
    /// [module docs](self).
    ///
    /// # Deviation
    ///
    /// Upstream's `sign(endpos - startpos)` is reproduced as `if pos2 >=
    /// pos1 { 1.0 } else { -1.0 }` rather than a 3-way sign (see the [module
    /// docs](self)'s "no incomplete-profile branch's `sign` ambiguity" note
    /// — every caller here passes `pos2 > pos1`).
    pub fn set_profile(&mut self, pos1: f64, pos2: f64) {
        self.start_pos = pos1;
        self.end_pos = pos2;
        let s = if pos2 >= pos1 { 1.0 } else { -1.0 };
        let dist = (pos2 - pos1).abs();

        let t_acc = self.max_vel / self.max_acc;
        let complete_dist = self.max_vel * t_acc;
        let (t1, t2) = if dist >= complete_dist {
            let cruise_time = (dist - complete_dist) / self.max_vel;
            (t_acc, t_acc + cruise_time)
        } else {
            let t1 = (dist / self.max_acc).sqrt();
            (t1, t1)
        };
        self.duration = t1 + t2;
        self.t1 = t1;
        self.t2 = t2;

        self.accel = s * self.max_acc;
        self.v_peak = self.accel * t1;
        self.pos_at_t1 = self.start_pos + 0.5 * self.v_peak * t1;
        self.pos_at_t2 = self.pos_at_t1 + self.v_peak * (t2 - t1);
    }

    /// Upstream `Duration`.
    pub fn duration(&self) -> f64 {
        self.duration
    }

    /// Position at `time` (clamped to `[start_pos, end_pos]` outside
    /// `[0, duration]`).
    ///
    /// # Not transcribed from `VelocityProfile_Trap::Pos`
    ///
    /// Evaluates the same three-phase (accelerate / cruise / decelerate)
    /// motion [`Self::set_profile`] derives, using local time within each
    /// phase rather than upstream's three globally-fitted quadratics — see
    /// that function's doc comment for the derivation and the equivalence
    /// argument. The decelerate phase's `-0.5 * accel * tau * tau` is the
    /// same SUVAT position formula as the accelerate phase, with the sign
    /// of the acceleration flipped and local time `tau = time - t2`.
    pub fn pos(&self, time: f64) -> f64 {
        if time < 0.0 {
            self.start_pos
        } else if time < self.t1 {
            self.start_pos + 0.5 * self.accel * time * time
        } else if time < self.t2 {
            self.pos_at_t1 + self.v_peak * (time - self.t1)
        } else if time <= self.duration {
            let tau = time - self.t2;
            self.pos_at_t2 + self.v_peak * tau - 0.5 * self.accel * tau * tau
        } else {
            self.end_pos
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    // -- set_profile: complete-profile boundary (velocity saturates) vs
    // incomplete (triangular) --

    #[test]
    fn set_profile_reaches_max_velocity_when_distance_is_long_enough() {
        let mut profile = VelocityProfileTrap::new(1.0, 1.0);
        profile.set_profile(0.0, 10.0);
        // Accelerate for max_vel/max_acc = 1s, then cruise, then decelerate
        // for 1s: t1 == 1.0 and t2 < duration strictly (a genuine cruise
        // phase exists).
        assert_relative_eq!(profile.t1, 1.0);
        assert!(profile.t2 > profile.t1);
        assert_relative_eq!(profile.pos(0.0), 0.0);
        assert_relative_eq!(profile.pos(profile.duration()), 10.0);
    }

    #[test]
    fn set_profile_is_triangular_when_distance_is_too_short_to_saturate() {
        let mut profile = VelocityProfileTrap::new(10.0, 1.0);
        profile.set_profile(0.0, 1.0);
        // Never reaches max_vel: t1 == t2 (no cruise phase).
        assert_relative_eq!(profile.t1, profile.t2);
        assert_relative_eq!(profile.pos(0.0), 0.0);
        assert_relative_eq!(profile.pos(profile.duration()), 1.0, epsilon = 1e-9);
    }

    // -- pos: before start and after duration clamp to the endpoints --

    #[test]
    fn pos_clamps_before_start_and_after_duration() {
        let mut profile = VelocityProfileTrap::new(1.0, 1.0);
        profile.set_profile(2.0, 5.0);
        assert_relative_eq!(profile.pos(-1.0), 2.0);
        assert_relative_eq!(profile.pos(profile.duration() + 1.0), 5.0);
    }

    // -- set_profile: the trapezoidal/triangular branches must agree
    // exactly at their shared boundary (distance == max_vel^2/max_acc) --

    #[test]
    fn set_profile_at_the_exact_saturation_boundary_has_no_cruise_phase() {
        // max_vel = 2, max_acc = 1 => complete_dist = max_vel^2/max_acc = 4.
        let mut profile = VelocityProfileTrap::new(2.0, 1.0);
        profile.set_profile(0.0, 4.0);
        // Exactly at the boundary the trapezoidal branch's cruise_time is
        // zero, so it must degenerate to the triangular branch's t1 == t2,
        // not leave a residual (possibly negative, from rounding) cruise
        // phase.
        assert_relative_eq!(profile.t1, profile.t2);
        assert_relative_eq!(profile.t1, 2.0);
        assert_relative_eq!(profile.pos(profile.duration()), 4.0);
    }

    // -- pos: continuity across each phase boundary (accel/cruise,
    // cruise/decel) --

    #[test]
    fn pos_is_continuous_across_the_accel_to_cruise_boundary() {
        let mut profile = VelocityProfileTrap::new(1.0, 1.0);
        profile.set_profile(0.0, 10.0);
        let t1 = profile.t1;
        // Evaluated from the accel-phase formula just below t1 and the
        // cruise-phase formula at t1 must agree.
        assert_relative_eq!(
            profile.start_pos + 0.5 * profile.accel * t1 * t1,
            profile.pos(t1),
            epsilon = 1e-9
        );
    }

    #[test]
    fn pos_is_continuous_across_the_cruise_to_decel_boundary() {
        let mut profile = VelocityProfileTrap::new(1.0, 1.0);
        profile.set_profile(0.0, 10.0);
        let t2 = profile.t2;
        assert_relative_eq!(
            profile.pos_at_t1 + profile.v_peak * (t2 - profile.t1),
            profile.pos(t2),
            epsilon = 1e-9
        );
    }

    // -- pos: the decel phase is the accel phase's SUVAT formula with the
    // acceleration sign flipped --

    #[test]
    fn pos_decel_phase_mirrors_the_accel_phase_velocity_magnitude() {
        let mut profile = VelocityProfileTrap::new(1.0, 1.0);
        profile.set_profile(0.0, 10.0);
        let dt = 1e-6;
        let accel_speed = (profile.pos(dt) - profile.pos(0.0)) / dt;
        let decel_speed =
            (profile.pos(profile.duration()) - profile.pos(profile.duration() - dt)) / dt;
        assert_relative_eq!(accel_speed, decel_speed, epsilon = 1e-4);
    }
}
