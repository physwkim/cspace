// Copyright (c) 2004-2005, Erwin Aertbelien, Div. PMA, Dep. of Mech. Eng., K.U.Leuven
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from orocos_kinematics_dynamics @ v1.5.1 (see
// `crates/moveit-state/src/dynamics.rs` for how this workspace pins and
// verifies that checkout against the oracle image's compiled `liborocos-kdl`):
//   orocos_kdl/src/velocityprofile_trap.hpp
//   orocos_kdl/src/velocityprofile_trap.cpp
// used by moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf's
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator.cpp
// (`TrajectoryGenerator::cartesianTrapVelocityProfile`).

//! Symmetric trapezoidal velocity profile ([`VelocityProfileTrap`]).
//!
//! Ported from `KDL::VelocityProfile_Trap`, the profile
//! `TrajectoryGenerator::cartesianTrapVelocityProfile` builds to time-
//! parametrize a Cartesian path's arc length for `LIN`/`CIRC` — distinct from
//! [`crate::velocity_profile::VelocityProfileAtrap`] (Pilz's own asymmetric
//! profile, used by `PTP`'s per-joint synchronization): this one has a single
//! acceleration magnitude shared by both ramps, matching KDL's own type.
//!
//! # Deviations from upstream
//!
//! - **Only `SetProfile`/`Duration`/`Pos` are ported.** Upstream's
//!   `SetProfileDuration`/`SetProfileVelocity`/`Vel`/`Acc`/`Write`/`Clone`
//!   exist to satisfy `KDL::VelocityProfile`'s polymorphic interface and
//!   upstream's own re-timing callers; `cartesianTrapVelocityProfile` (this
//!   port's only caller of this type) only ever calls `SetProfile` once and
//!   samples `Pos`, so the rest have no reader here — see this crate's
//!   `deny(warnings)` policy on dead code, and
//!   [`crate::velocity_profile::VelocityProfileAtrap`]'s own doc for the same
//!   "no `KDL::VelocityProfile` base class" reasoning.
//! - **No incomplete-profile branch's `sign` ambiguity.** Upstream's
//!   `SetProfile` computes `s = sign(endpos - startpos)`, which is `0` when
//!   `endpos == startpos` (`KDL::sign` returns `0` for a zero argument) —
//!   `cartesianTrapVelocityProfile` always calls `SetProfile(0, length)` with
//!   `length` at least `f64::EPSILON` (its own zero-length fallback), so
//!   `endpos > startpos` always holds for every caller this port has; `sign`
//!   is reproduced as a plain `if` rather than a general 3-way function.

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

    // Phase coefficients: pos(t) = a1/b1/c1 + t * (a2/b2/c2 + t * a3/b3/c3),
    // local to each phase.
    a1: f64,
    a2: f64,
    a3: f64,
    b1: f64,
    b2: f64,
    b3: f64,
    c1: f64,
    c2: f64,
    c3: f64,

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
            a1: 0.0,
            a2: 0.0,
            a3: 0.0,
            b1: 0.0,
            b2: 0.0,
            b3: 0.0,
            c1: 0.0,
            c2: 0.0,
            c3: 0.0,
            duration: 0.0,
            t1: 0.0,
            t2: 0.0,
        }
    }

    /// Plan a profile from `pos1` to `pos2` at this profile's velocity/
    /// acceleration limits. Upstream `SetProfile`.
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
        let mut t1 = self.max_vel / self.max_acc;
        let s = if pos2 >= pos1 { 1.0 } else { -1.0 };
        let delta_x1 = s * self.max_acc * t1 * t1 / 2.0;
        let delta_t = (self.end_pos - self.start_pos - 2.0 * delta_x1) / (s * self.max_vel);
        let t2;
        if delta_t > 0.0 {
            self.duration = 2.0 * t1 + delta_t;
            t2 = self.duration - t1;
        } else {
            t1 = ((self.end_pos - self.start_pos) / s / self.max_acc).sqrt();
            self.duration = t1 * 2.0;
            t2 = t1;
        }
        self.t1 = t1;
        self.t2 = t2;

        self.a3 = s * self.max_acc / 2.0;
        self.a2 = 0.0;
        self.a1 = self.start_pos;

        self.b3 = 0.0;
        self.b2 = self.a2 + 2.0 * self.a3 * t1 - 2.0 * self.b3 * t1;
        self.b1 = self.a1 + t1 * (self.a2 + self.a3 * t1) - t1 * (self.b2 + t1 * self.b3);

        self.c3 = -s * self.max_acc / 2.0;
        self.c2 = self.b2 + 2.0 * self.b3 * t2 - 2.0 * self.c3 * t2;
        self.c1 = self.b1 + t2 * (self.b2 + self.b3 * t2) - t2 * (self.c2 + t2 * self.c3);
    }

    /// Upstream `Duration`.
    pub fn duration(&self) -> f64 {
        self.duration
    }

    /// Upstream `Pos`.
    pub fn pos(&self, time: f64) -> f64 {
        if time < 0.0 {
            self.start_pos
        } else if time < self.t1 {
            self.a1 + time * (self.a2 + self.a3 * time)
        } else if time < self.t2 {
            self.b1 + time * (self.b2 + self.b3 * time)
        } else if time <= self.duration {
            self.c1 + time * (self.c2 + self.c3 * time)
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
}
