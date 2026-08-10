// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/velocity_profile_atrap.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/velocity_profile_atrap.cpp

//! Asymmetric trapezoidal velocity profile ([`VelocityProfileAtrap`]).
//!
//! Ported from upstream's `VelocityProfileATrap`, a `KDL::VelocityProfile`
//! subclass. A profile is a 1-D motion between `pos1` and `pos2` made of up
//! to three quadratic phases (accelerate / constant / decelerate), stored as
//! per-phase coefficients `pos(t) = c1 + t * (c2 + c3 * t)` plus per-phase
//! durations.
//!
//! # Deviations from upstream
//!
//! - **No `KDL::VelocityProfile` base class.** This port has no polymorphic
//!   `VelocityProfile` trait hierarchy (nothing else in this crate needs to
//!   hold a `Box<dyn VelocityProfile>`), so [`VelocityProfileAtrap`] is a
//!   plain struct, not a trait implementation.
//! - **`Clone()` is not ported as a recompute.** Upstream's
//!   `VelocityProfileATrap::Clone()` returns a *new* profile built by calling
//!   `setProfileAllDurations` on the source's own phase durations and
//!   positions — a recompute, not a field copy, because it exists to satisfy
//!   `KDL::VelocityProfile*`'s polymorphic clone contract. Since that
//!   contract does not exist here, `#[derive(Clone)]` gives an exact field
//!   copy, which is strictly more precise than upstream's recompute (the
//!   recompute can introduce floating-point noise that a plain copy cannot).
//!   [`VelocityProfileAtrap::set_profile_all_durations`] is still ported
//!   in full and its self-consistency (feeding a profile's own durations and
//!   endpoints back into it reproduces the same profile) is what upstream's
//!   `Test_Clone` actually exercises; see the `set_profile_all_durations_is_self_consistent`
//!   test below.
//! - **`Write`/`operator<<` are not ported.** They only format state to a
//!   stream for logging (upstream even excludes them from coverage with
//!   `LCOV_EXCL_START`/`STOP`); `#[derive(Debug)]` covers the same need.
//! - **`assert()` becomes `debug_assert!()`.** Upstream's
//!   `setProfileAllDurations` asserts `duration1 > 0` and `duration3 > 0`
//!   with a plain C `assert()`, which upstream builds compile out entirely in
//!   release (`NDEBUG`). `debug_assert!` has the same "checked in debug,
//!   absent in release" behaviour.
//!
//! `KDL::epsilon` (`orocos_kdl/src/utilities/utility.cxx`) is `1e-6`; it is
//! reproduced here as [`KDL_EPSILON`] rather than imported, since this crate
//! does not depend on KDL.

/// `KDL::epsilon`, from `orocos_kdl/src/utilities/utility.cxx`.
pub const KDL_EPSILON: f64 = 1e-6;

/// An asymmetric trapezoidal (accelerate / constant / decelerate) velocity
/// profile between two 1-D positions.
///
/// See the [module docs](self) for the upstream mapping and deviations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityProfileAtrap {
    max_vel: f64,
    max_acc: f64,
    max_dec: f64,
    start_pos: f64,
    end_pos: f64,
    start_vel: f64,

    // Phase coefficients: pos(t) = c1 + t * (c2 + c3 * t), local to each phase.
    a1: f64,
    a2: f64,
    a3: f64,
    b1: f64,
    b2: f64,
    b3: f64,
    c1: f64,
    c2: f64,
    c3: f64,

    t_a: f64,
    t_b: f64,
    t_c: f64,
}

impl VelocityProfileAtrap {
    /// Build a profile with the given (always-positive) velocity/acceleration/
    /// deceleration limits. Negative inputs are reflected, matching upstream's
    /// `fabs()` in the constructor.
    pub fn new(max_vel: f64, max_acc: f64, max_dec: f64) -> Self {
        Self {
            max_vel: max_vel.abs(),
            max_acc: max_acc.abs(),
            max_dec: max_dec.abs(),
            start_pos: 0.0,
            end_pos: 0.0,
            start_vel: 0.0,
            a1: 0.0,
            a2: 0.0,
            a3: 0.0,
            b1: 0.0,
            b2: 0.0,
            b3: 0.0,
            c1: 0.0,
            c2: 0.0,
            c3: 0.0,
            t_a: 0.0,
            t_b: 0.0,
            t_c: 0.0,
        }
    }

    /// Compute the fastest profile from `pos1` to `pos2` obeying the
    /// configured limits.
    pub fn set_profile(&mut self, pos1: f64, pos2: f64) {
        self.start_pos = pos1;
        self.end_pos = pos2;
        self.start_vel = 0.0;

        if self.start_pos == self.end_pos {
            self.set_empty_profile();
            return;
        }

        let s = sign(self.end_pos - self.start_pos);
        let dis = (self.end_pos - self.start_pos).abs();
        let min_dis_max_vel = 0.5 * self.max_vel * self.max_vel / self.max_acc
            + 0.5 * self.max_vel * self.max_vel / self.max_dec;

        if dis > min_dis_max_vel {
            // Max velocity is reached: accelerate, cruise, decelerate.
            self.a1 = self.start_pos;
            self.a2 = 0.0;
            self.a3 = s * self.max_acc / 2.0;
            self.t_a = self.max_vel / self.max_acc;

            self.b1 = self.a1 + self.a3 * self.t_a * self.t_a;
            self.b2 = s * self.max_vel;
            self.b3 = 0.0;
            self.t_b = (dis - min_dis_max_vel) / self.max_vel;

            self.c1 = self.b1 + self.b2 * self.t_b;
            self.c2 = s * self.max_vel;
            self.c3 = -s * self.max_dec / 2.0;
            self.t_c = self.max_vel / self.max_dec;
        } else {
            // Max velocity is not reached: no constant-velocity phase.
            let new_vel = s
                * (2.0 * dis * self.max_acc * self.max_dec / (self.max_acc + self.max_dec)).sqrt();

            self.a1 = self.start_pos;
            self.a2 = 0.0;
            self.a3 = s * self.max_acc / 2.0;
            self.t_a = new_vel.abs() / self.max_acc;

            self.b1 = self.a1 + self.a3 * self.t_a * self.t_a;
            self.b2 = new_vel;
            self.b3 = 0.0;
            self.t_b = 0.0;

            self.c1 = self.b1;
            self.c2 = new_vel;
            self.c3 = -s * self.max_dec / 2.0;
            self.t_c = new_vel.abs() / self.max_dec;
        }
    }

    /// Compute the fastest profile, then scale it to last exactly `duration`
    /// (ignored if `duration` is shorter than the fastest case, or if
    /// `pos1 == pos2` so that there is no phase structure to scale).
    ///
    /// # Deviations from upstream
    ///
    /// Upstream `SetProfileDuration` (`velocity_profile_atrap.cpp:129-149`)
    /// guards only the too-short case. A zero-length profile passes that
    /// guard for every `duration >= 0`, and the `ratio` it then scales by is
    /// `0.0 / duration` — zero, so the `t_* /= ratio` lines below evaluate
    /// `0.0 / 0.0` and leave all three phase durations `NaN` (`ratio` is
    /// itself `NaN` when `duration` is zero). Both boundaries are covered by
    /// the same rule as the too-short check: a duration that scaling cannot
    /// reach leaves the fastest profile in place.
    pub fn set_profile_duration(&mut self, pos1: f64, pos2: f64, duration: f64) {
        self.set_profile(pos1, pos2);

        if self.duration() > duration || self.duration() <= 0.0 {
            return;
        }

        let ratio = self.duration() / duration;
        self.a2 *= ratio;
        self.a3 *= ratio * ratio;
        self.b2 *= ratio;
        self.b3 *= ratio * ratio;
        self.c2 *= ratio;
        self.c3 *= ratio * ratio;
        self.t_a /= ratio;
        self.t_b /= ratio;
        self.t_c /= ratio;
    }

    /// Compute a profile with exactly the given per-phase durations. Returns
    /// `false` (leaving the fastest profile from the internal `set_profile`
    /// call in place) if the combination is faster than physically possible
    /// or violates a velocity/acceleration/deceleration limit.
    pub fn set_profile_all_durations(
        &mut self,
        pos1: f64,
        pos2: f64,
        accel_duration: f64,
        const_duration: f64,
        decel_duration: f64,
    ) -> bool {
        self.set_profile(pos1, pos2);

        debug_assert!(accel_duration > 0.0);
        debug_assert!(decel_duration > 0.0);

        if self.duration() - (accel_duration + const_duration + decel_duration) > KDL_EPSILON {
            return false;
        }

        let s = sign(self.end_pos - self.start_pos);
        let dis = (self.end_pos - self.start_pos).abs();
        let new_vel = s * dis / (const_duration + accel_duration / 2.0 + decel_duration / 2.0);
        let new_acc = new_vel / accel_duration;
        let new_dec = -new_vel / decel_duration;
        if (new_vel.abs() - self.max_vel > KDL_EPSILON)
            || (new_acc.abs() - self.max_acc > KDL_EPSILON)
            || (new_dec.abs() - self.max_dec > KDL_EPSILON)
        {
            return false;
        }

        self.start_pos = pos1;
        self.end_pos = pos2;

        self.a1 = self.start_pos;
        self.a2 = 0.0;
        self.a3 = new_acc / 2.0;
        self.t_a = accel_duration;

        self.b1 = self.a1 + self.a3 * self.t_a * self.t_a;
        self.b2 = new_vel;
        self.b3 = 0.0;
        self.t_b = const_duration;

        self.c1 = self.b1 + self.b2 * self.t_b;
        self.c2 = new_vel;
        self.c3 = new_dec / 2.0;
        self.t_c = decel_duration;

        true
    }

    /// Compute a profile starting from a nonzero velocity `vel1`. Only
    /// supports `vel1` in the same direction as `pos2 - pos1` (matching
    /// upstream's documented restriction to the live-control use case);
    /// returns `false` for the opposite-direction case instead.
    pub fn set_profile_start_velocity(&mut self, pos1: f64, pos2: f64, vel1: f64) -> bool {
        if vel1 == 0.0 {
            self.set_profile(pos1, pos2);
            return true;
        }

        let s = sign(pos2 - pos1);
        if s * vel1 <= 0.0 {
            return false;
        }

        self.start_pos = pos1;
        self.end_pos = pos2;
        self.start_vel = vel1;

        let min_brake_dis = 0.5 * vel1 * vel1 / self.max_dec;
        let min_dis_max_vel =
            0.5 * (self.max_vel - self.start_vel) * (self.max_vel + self.start_vel) / self.max_acc
                + 0.5 * self.max_vel * self.max_vel / self.max_dec;
        let dis = (self.end_pos - self.start_pos).abs();

        if dis <= min_brake_dis {
            // Brake to zero, accelerate in the opposite direction, decelerate.
            self.t_a = (self.start_vel / self.max_dec).abs();
            self.a1 = self.start_pos;
            self.a2 = self.start_vel;
            self.a3 = -0.5 * s * self.max_dec;

            let new_vel = -s
                * (2.0 * (min_brake_dis - dis).abs() * self.max_acc * self.max_dec
                    / (self.max_acc + self.max_dec))
                    .sqrt();

            self.t_b = (new_vel / self.max_acc).abs();
            self.b1 = self.a1 + self.a2 * self.t_a + self.a3 * self.t_a * self.t_a;
            self.b2 = 0.0;
            self.b3 = -s * 0.5 * self.max_acc;

            self.t_c = (new_vel / self.max_dec).abs();
            self.c1 = self.b1 + self.b2 * self.t_b + self.b3 * self.t_b * self.t_b;
            self.c2 = new_vel;
            self.c3 = 0.5 * s * self.max_dec;
        } else if dis <= min_dis_max_vel {
            // Accelerate to a reduced peak, no constant-velocity phase, decelerate.
            let new_vel = s
                * ((dis + 0.5 * self.start_vel * self.start_vel / self.max_acc)
                    * 2.0
                    * self.max_acc
                    * self.max_dec
                    / (self.max_acc + self.max_dec))
                    .sqrt();

            self.t_a = (new_vel - self.start_vel).abs() / self.max_acc;
            self.a1 = self.start_pos;
            self.a2 = self.start_vel;
            self.a3 = 0.5 * s * self.max_acc;

            self.t_b = 0.0;
            self.b1 = self.a1 + self.a2 * self.t_a + self.a3 * self.t_a * self.t_a;
            self.b2 = 0.0;
            self.b3 = 0.0;

            self.t_c = (new_vel / self.max_dec).abs();
            self.c1 = self.b1;
            self.c2 = new_vel;
            self.c3 = -0.5 * s * self.max_dec;
        } else {
            // Full trapezoid: accelerate to max velocity, cruise, decelerate.
            self.t_a = (self.max_vel - self.start_vel).abs() / self.max_acc;
            self.a1 = self.start_pos;
            self.a2 = self.start_vel;
            self.a3 = 0.5 * s * self.max_acc;

            self.t_b = (dis - min_dis_max_vel) / self.max_vel;
            self.b1 = self.a1 + self.a2 * self.t_a + self.a3 * self.t_a * self.t_a;
            self.b2 = self.max_vel;
            self.b3 = 0.0;

            self.t_c = self.max_vel / self.max_dec;
            self.c1 = self.b1 + self.b2 * self.t_b + self.b3 * self.t_b * self.t_b;
            self.c2 = self.max_vel;
            self.c3 = -0.5 * s * self.max_dec;
        }

        true
    }

    /// Duration of the acceleration phase.
    pub fn first_phase_duration(&self) -> f64 {
        self.t_a
    }

    /// Duration of the constant-velocity phase.
    pub fn second_phase_duration(&self) -> f64 {
        self.t_b
    }

    /// Duration of the deceleration phase.
    pub fn third_phase_duration(&self) -> f64 {
        self.t_c
    }

    /// Total duration of the profile.
    pub fn duration(&self) -> f64 {
        self.t_a + self.t_b + self.t_c
    }

    /// Position at `time`. Clamped to `start_pos`/`end_pos` outside `[0,
    /// duration()]`.
    pub fn pos(&self, time: f64) -> f64 {
        if time < 0.0 {
            self.start_pos
        } else if time < self.t_a {
            self.a1 + time * (self.a2 + self.a3 * time)
        } else if time < (self.t_a + self.t_b) {
            let t = time - self.t_a;
            self.b1 + t * (self.b2 + self.b3 * t)
        } else if time <= (self.t_a + self.t_b + self.t_c) {
            let t = time - self.t_a - self.t_b;
            self.c1 + t * (self.c2 + self.c3 * t)
        } else {
            self.end_pos
        }
    }

    /// Velocity at `time`. `start_vel` before the profile starts, `0` after
    /// it ends.
    pub fn vel(&self, time: f64) -> f64 {
        if time < 0.0 {
            self.start_vel
        } else if time < self.t_a {
            self.a2 + 2.0 * self.a3 * time
        } else if time < (self.t_a + self.t_b) {
            self.b2 + 2.0 * self.b3 * (time - self.t_a)
        } else if time <= (self.t_a + self.t_b + self.t_c) {
            self.c2 + 2.0 * self.c3 * (time - self.t_a - self.t_b)
        } else {
            0.0
        }
    }

    /// Acceleration at `time`. `0` at and before `time == 0` and after the
    /// profile ends (matching upstream's `time <= 0` boundary, which differs
    /// from [`Self::vel`]'s `time < 0`).
    pub fn acc(&self, time: f64) -> f64 {
        if time <= 0.0 {
            0.0
        } else if time <= self.t_a {
            2.0 * self.a3
        } else if time <= (self.t_a + self.t_b) {
            2.0 * self.b3
        } else if time <= (self.t_a + self.t_b + self.t_c) {
            2.0 * self.c3
        } else {
            0.0
        }
    }

    fn set_empty_profile(&mut self) {
        self.a1 = self.end_pos;
        self.a2 = 0.0;
        self.a3 = 0.0;
        self.b1 = self.end_pos;
        self.b2 = 0.0;
        // b3 is intentionally left unchanged, matching upstream: with
        // t_a = t_b = t_c = 0, the b-phase interval [t_a, t_a + t_b) is empty
        // and b3 is never read by pos()/vel()/acc(), so its stale value is
        // inert.
        self.c1 = self.end_pos;
        self.c2 = 0.0;
        self.c3 = 0.0;

        self.t_a = 0.0;
        self.t_b = 0.0;
        self.t_c = 0.0;
    }
}

/// `sign(x)` as used throughout upstream: `(x > 0) - (x < 0)`, i.e. `-1.0`,
/// `0.0` or `1.0`.
fn sign(x: f64) -> f64 {
    ((x > 0.0) as i32 - (x < 0.0) as i32) as f64
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const EPSILON: f64 = 1.0e-10;

    fn assert_near(actual: f64, expected: f64) {
        assert_relative_eq!(actual, expected, epsilon = EPSILON, max_relative = EPSILON);
    }

    /// Full trapezoid: max velocity is reached.
    #[test]
    fn set_profile_reaches_max_velocity() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        vp.set_profile(3.0, 35.0);

        assert_near(vp.duration(), 11.0);

        assert_near(vp.pos(-1.0), 3.0);
        assert_near(vp.vel(-1.0), 0.0);
        assert_near(vp.acc(-1.0), 0.0);

        assert_near(vp.pos(0.0), 3.0);
        assert_near(vp.vel(0.0), 0.0);
        assert_near(vp.acc(0.0), 0.0);

        assert_near(vp.pos(1.0), 4.0);
        assert_near(vp.vel(1.0), 2.0);
        assert_near(vp.acc(1.0), 2.0);

        assert_near(vp.pos(2.0), 7.0);
        assert_near(vp.vel(2.0), 4.0);
        assert_near(vp.acc(2.0), 2.0);

        assert_near(vp.pos(4.5), 17.0);
        assert_near(vp.vel(4.5), 4.0);
        assert_near(vp.acc(4.5), 0.0);

        assert_near(vp.pos(7.0), 27.0);
        assert_near(vp.vel(7.0), 4.0);
        assert_near(vp.acc(7.0), 0.0);

        assert_near(vp.pos(9.0), 33.0);
        assert_near(vp.vel(9.0), 2.0);
        assert_near(vp.acc(9.0), -1.0);

        assert_near(vp.pos(11.0), 35.0);
        assert_near(vp.vel(11.0), 0.0);
        assert_near(vp.acc(11.0), -1.0);

        assert_near(vp.pos(12.0), 35.0);
        assert_near(vp.vel(12.0), 0.0);
        assert_near(vp.acc(12.0), 0.0);
    }

    /// Boundary: distance is exactly enough to just arrive at max velocity
    /// (`t_b == 0` but the "reaches max vel" branch is still taken).
    #[test]
    fn set_profile_just_arrives_at_max_velocity() {
        let mut vp = VelocityProfileAtrap::new(6.0, 2.0, 1.5);
        vp.set_profile(5.0, 26.0);

        assert_near(vp.duration(), 7.0);

        assert_near(vp.pos(1.5), 7.25);
        assert_near(vp.vel(1.5), 3.0);
        assert_near(vp.acc(1.5), 2.0);

        assert_near(vp.pos(3.0), 14.0);
        assert_near(vp.vel(3.0), 6.0);
        assert_near(vp.acc(3.0), 2.0);

        assert_near(vp.pos(5.0), 23.0);
        assert_near(vp.vel(5.0), 3.0);
        assert_near(vp.acc(5.0), -1.5);

        assert_near(vp.pos(7.0), 26.0);
        assert_near(vp.vel(7.0), 0.0);
        assert_near(vp.acc(7.0), -1.5);

        assert_near(vp.pos(8.0), 26.0);
        assert_near(vp.vel(8.0), 0.0);
        assert_near(vp.acc(8.0), 0.0);
    }

    /// Triangular: distance is too short to reach max velocity, so there is
    /// no constant-velocity phase.
    #[test]
    fn set_profile_cannot_reach_max_velocity_is_triangular() {
        let mut vp = VelocityProfileAtrap::new(6.0, 2.0, 1.0);
        vp.set_profile(5.0, 17.0);

        assert_near(vp.duration(), 6.0);
        assert_near(vp.second_phase_duration(), 0.0);

        assert_near(vp.pos(1.0), 6.0);
        assert_near(vp.vel(1.0), 2.0);
        assert_near(vp.acc(1.0), 2.0);

        assert_near(vp.pos(2.0), 9.0);
        assert_near(vp.vel(2.0), 4.0);
        assert_near(vp.acc(2.0), 2.0);

        assert_near(vp.pos(4.0), 15.0);
        assert_near(vp.vel(4.0), 2.0);
        assert_near(vp.acc(4.0), -1.0);

        assert_near(vp.pos(6.0), 17.0);
        assert_near(vp.vel(6.0), 0.0);
        assert_near(vp.acc(6.0), -1.0);

        assert_near(vp.pos(7.0), 17.0);
        assert_near(vp.vel(7.0), 0.0);
        assert_near(vp.acc(7.0), 0.0);
    }

    /// Boundary: zero-distance goal produces an empty (all-zero-duration)
    /// profile.
    #[test]
    fn set_profile_zero_distance_is_empty() {
        let mut vp = VelocityProfileAtrap::new(6.0, 2.0, 1.0);
        vp.set_profile(5.0, 5.0);

        assert_near(vp.duration(), 0.0);
        assert_near(vp.pos(-1.0), 5.0);
        assert_near(vp.vel(-1.0), 0.0);
        assert_near(vp.acc(-1.0), 0.0);
        assert_near(vp.pos(0.0), 5.0);
        assert_near(vp.vel(0.0), 0.0);
        assert_near(vp.acc(0.0), 0.0);
    }

    /// Requesting a duration shorter than the fastest possible profile is a
    /// no-op: the fastest profile is kept.
    #[test]
    fn set_profile_duration_below_fastest_is_ignored() {
        let mut vp1 = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        let mut vp2 = vp1;

        vp1.set_profile_duration(3.0, 35.0, f64::EPSILON);
        let fastest_duration = vp1.duration();

        vp2.set_profile_duration(3.0, 35.0, fastest_duration);

        assert_eq!(vp1, vp2);
    }

    /// A zero-length profile (`pos1 == pos2`) has no phase structure to
    /// stretch, so requesting a duration for one must leave it alone. Both
    /// boundaries of the ratio reach `0.0 / 0.0` without the guard: a
    /// positive `duration` gives `ratio == 0.0` and then `t_a /= 0.0`, and a
    /// zero `duration` makes `ratio` itself `NaN`.
    #[test]
    fn zero_length_profile_is_not_scaled_into_nan() {
        for duration in [22.0, 0.0] {
            let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
            vp.set_profile_duration(3.0, 3.0, duration);

            assert!(
                vp.duration().is_finite(),
                "zero-length profile scaled to {duration} left a non-finite \
                 duration: {}",
                vp.duration()
            );
            assert_eq!(vp.duration(), 0.0);
            assert!(vp.pos(1.0).is_finite());
        }
    }

    /// Scaling to a longer duration stretches every phase by the same ratio.
    #[test]
    fn set_profile_duration_scales_all_phases() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        vp.set_profile_duration(3.0, 35.0, 22.0);

        assert_near(vp.duration(), 22.0);

        assert_near(vp.pos(2.0), 4.0);
        assert_near(vp.vel(2.0), 1.0);
        assert_near(vp.acc(2.0), 0.5);

        assert_near(vp.pos(4.0), 7.0);
        assert_near(vp.vel(4.0), 2.0);
        assert_near(vp.acc(4.0), 0.5);

        assert_near(vp.pos(9.0), 17.0);
        assert_near(vp.vel(9.0), 2.0);
        assert_near(vp.acc(9.0), 0.0);

        assert_near(vp.pos(14.0), 27.0);
        assert_near(vp.vel(14.0), 2.0);
        assert_near(vp.acc(14.0), 0.0);

        assert_near(vp.pos(18.0), 33.0);
        assert_near(vp.vel(18.0), 1.0);
        assert_near(vp.acc(18.0), -0.25);

        assert_near(vp.pos(22.0), 35.0);
        assert_near(vp.vel(22.0), 0.0);
        assert_near(vp.acc(22.0), -0.25);

        assert_near(vp.pos(23.0), 35.0);
        assert_near(vp.vel(23.0), 0.0);
        assert_near(vp.acc(23.0), 0.0);
    }

    /// A faster-than-possible explicit-durations request leaves the fastest
    /// profile (from the internal `set_profile` call) untouched and reports
    /// failure.
    #[test]
    fn set_profile_all_durations_below_fastest_is_rejected() {
        let mut vp1 = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        let mut vp2 = vp1;

        vp1.set_profile(3.0, 35.0);
        let fastest_duration = vp1.duration();

        assert!(!vp2.set_profile_all_durations(
            3.0,
            35.0,
            fastest_duration / 4.0,
            fastest_duration / 4.0,
            fastest_duration / 4.0
        ));

        assert_eq!(vp1, vp2);
    }

    /// Explicit valid durations produce the exact phase coefficients upstream
    /// asserts on.
    #[test]
    fn set_profile_all_durations_valid_combination() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(vp.set_profile_all_durations(3.0, 35.0, 3.0, 4.0, 5.0));

        assert_near(vp.duration(), 12.0);

        assert_near(vp.pos(2.0), 3.0 + 8.0 / 3.0);
        assert_near(vp.vel(2.0), 8.0 / 3.0);
        assert_near(vp.acc(2.0), 4.0 / 3.0);

        assert_near(vp.pos(3.0), 9.0);
        assert_near(vp.vel(3.0), 4.0);
        assert_near(vp.acc(3.0), 4.0 / 3.0);

        assert_near(vp.pos(5.0), 17.0);
        assert_near(vp.vel(5.0), 4.0);
        assert_near(vp.acc(5.0), 0.0);

        assert_near(vp.pos(7.0), 25.0);
        assert_near(vp.vel(7.0), 4.0);
        assert_near(vp.acc(7.0), 0.0);

        assert_near(vp.pos(9.0), 31.4);
        assert_near(vp.vel(9.0), 2.4);
        assert_near(vp.acc(9.0), -0.8);

        assert_near(vp.pos(12.0), 35.0);
        assert_near(vp.vel(12.0), 0.0);
        assert_near(vp.acc(12.0), -0.8);

        assert_near(vp.pos(13.0), 35.0);
        assert_near(vp.vel(13.0), 0.0);
        assert_near(vp.acc(13.0), 0.0);
    }

    /// Each of the three explicit-duration combinations individually
    /// violates one limit (velocity, acceleration, deceleration).
    #[test]
    fn set_profile_all_durations_rejects_each_limit_violation() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(!vp.set_profile_all_durations(3.0, 35.0, 3.0, 3.0, 5.0)); // velocity
        assert!(!vp.set_profile_all_durations(3.0, 35.0, 1.0, 4.0, 7.0)); // acceleration
        assert!(!vp.set_profile_all_durations(3.0, 35.0, 7.0, 4.0, 1.0)); // deceleration
    }

    /// Feeding a profile's own phase durations and endpoints back through
    /// `set_profile_all_durations` reproduces the same profile — this is the
    /// self-consistency upstream's `Test_Clone` actually exercises (see the
    /// module-level deviation note on why `Clone()` itself is not ported).
    #[test]
    fn set_profile_all_durations_is_self_consistent() {
        let mut vp = VelocityProfileAtrap::new(4.0, 1.0, 1.0);
        assert!(vp.set_profile_all_durations(0.0, 10.0, 10.0, 10.0, 10.0));

        let mut round_tripped = VelocityProfileAtrap::new(4.0, 1.0, 1.0);
        assert!(round_tripped.set_profile_all_durations(
            0.0,
            10.0,
            vp.first_phase_duration(),
            vp.second_phase_duration(),
            vp.third_phase_duration(),
        ));

        assert_eq!(vp, round_tripped);
    }

    /// Zero start velocity behaves exactly like `set_profile`.
    #[test]
    fn set_profile_start_velocity_zero_matches_set_profile() {
        let mut vp1 = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        let mut vp2 = vp1;

        vp1.set_profile(1.0, 2.0);
        assert!(vp2.set_profile_start_velocity(1.0, 2.0, 0.0));

        assert_eq!(vp1, vp2);
    }

    /// Start velocity opposite the travel direction is rejected.
    #[test]
    fn set_profile_start_velocity_opposite_direction_is_rejected() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(!vp.set_profile_start_velocity(3.0, 5.0, -1.0));
    }

    /// Sub-case: distance is short enough that the profile is pure
    /// deceleration to a stop (no acceleration or constant phase).
    #[test]
    fn set_profile_start_velocity_pure_deceleration() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(vp.set_profile_start_velocity(3.0, 5.0, 2.0));

        assert_near(vp.duration(), 2.0);
        assert_near(vp.first_phase_duration(), 2.0);
        assert_near(vp.second_phase_duration(), 0.0);
        assert_near(vp.third_phase_duration(), 0.0);

        assert_near(vp.pos(1.0), 4.5);
        assert_near(vp.vel(1.0), 1.0);
        assert_near(vp.acc(1.0), -1.0);

        assert_near(vp.pos(2.0), 5.0);
        assert_near(vp.vel(2.0), 0.0);
        assert_near(vp.acc(2.0), -1.0);

        assert_near(vp.pos(3.0), 5.0);
        assert_near(vp.vel(3.0), 0.0);
        assert_near(vp.acc(3.0), 0.0);
    }

    /// Sub-case: distance is too short even to brake cleanly — the profile
    /// brakes to zero, accelerates in the opposite direction, then
    /// decelerates back to zero at the goal.
    #[test]
    fn set_profile_start_velocity_brake_then_reverse() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(vp.set_profile_start_velocity(3.0, 4.0, 2.0));

        let sqrt_third = (1.0_f64 / 3.0).sqrt();
        assert_near(vp.duration(), 2.0 + 3.0 * sqrt_third);
        assert_near(vp.first_phase_duration(), 2.0);
        assert_near(vp.second_phase_duration(), sqrt_third);
        assert_near(vp.third_phase_duration(), 2.0 * sqrt_third);

        assert_near(vp.pos(2.0), 5.0);
        assert_near(vp.vel(2.0), 0.0);
        assert_near(vp.acc(2.0), -1.0);

        assert_near(vp.pos(2.1), 4.99);
        assert_near(vp.vel(2.1), -0.2);
        assert_near(vp.acc(2.1), -2.0);

        assert_near(vp.pos(2.0 + sqrt_third), 5.0 - 1.0 / 3.0);
        assert_near(vp.vel(2.0 + sqrt_third), -2.0 * sqrt_third);
        assert_near(vp.acc(2.0 + sqrt_third), -2.0);

        assert_near(vp.pos(2.0 + 3.0 * sqrt_third), 4.0);
        assert_near(vp.vel(2.0 + 3.0 * sqrt_third), 0.0);
        assert_near(vp.acc(2.0 + 3.0 * sqrt_third), 1.0);

        assert_near(vp.pos(5.0), 4.0);
        assert_near(vp.vel(5.0), 0.0);
        assert_near(vp.acc(5.0), 0.0);
    }

    /// Sub-case: accelerate to a reduced peak then decelerate, with no
    /// constant-velocity phase (`second_phase_duration() == 0`).
    #[test]
    fn set_profile_start_velocity_accelerate_then_decelerate_no_cruise() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(vp.set_profile_start_velocity(3.0, 14.0, 2.0));

        assert_near(vp.duration(), 5.0);
        assert_near(vp.first_phase_duration(), 1.0);
        assert_near(vp.second_phase_duration(), 0.0);
        assert_near(vp.third_phase_duration(), 4.0);

        assert_near(vp.pos(1.0), 6.0);
        assert_near(vp.vel(1.0), 4.0);
        assert_near(vp.acc(1.0), 2.0);

        assert_near(vp.pos(2.0), 9.5);
        assert_near(vp.vel(2.0), 3.0);
        assert_near(vp.acc(2.0), -1.0);

        assert_near(vp.pos(3.0), 12.0);
        assert_near(vp.vel(3.0), 2.0);
        assert_near(vp.acc(3.0), -1.0);

        assert_near(vp.pos(5.0), 14.0);
        assert_near(vp.vel(5.0), 0.0);
        assert_near(vp.acc(5.0), -1.0);
    }

    /// Sub-case: full trapezoid (accelerate, cruise at max velocity,
    /// decelerate).
    #[test]
    fn set_profile_start_velocity_full_trapezoid() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(vp.set_profile_start_velocity(3.0, 18.0, 2.0));

        assert_near(vp.duration(), 6.0);
        assert_near(vp.first_phase_duration(), 1.0);
        assert_near(vp.second_phase_duration(), 1.0);
        assert_near(vp.third_phase_duration(), 4.0);

        assert_near(vp.pos(2.0), 10.0);
        assert_near(vp.vel(2.0), 4.0);
        assert_near(vp.acc(2.0), 0.0);

        assert_near(vp.pos(3.0), 13.5);
        assert_near(vp.vel(3.0), 3.0);
        assert_near(vp.acc(3.0), -1.0);

        assert_near(vp.pos(4.0), 16.0);
        assert_near(vp.vel(4.0), 2.0);
        assert_near(vp.acc(4.0), -1.0);

        assert_near(vp.pos(6.0), 18.0);
        assert_near(vp.vel(6.0), 0.0);
        assert_near(vp.acc(6.0), -1.0);
    }

    /// Boundary: start velocity already above the requested peak — the
    /// acceleration phase collapses to zero duration and the profile begins
    /// directly with the constant phase.
    #[test]
    fn set_profile_start_velocity_no_accel_phase() {
        let mut vp = VelocityProfileAtrap::new(4.0, 2.0, 1.0);
        assert!(vp.set_profile_start_velocity(3.0, 15.0, 4.0));

        assert_near(vp.duration(), 5.0);
        assert_near(vp.first_phase_duration(), 0.0);
        assert_near(vp.second_phase_duration(), 1.0);
        assert_near(vp.third_phase_duration(), 4.0);

        assert_near(vp.pos(-1.0), 3.0);
        assert_near(vp.vel(-1.0), 4.0);

        assert_near(vp.pos(1.0), 7.0);
        assert_near(vp.vel(1.0), 4.0);
        assert_near(vp.acc(1.0), 0.0);

        assert_near(vp.pos(3.0), 13.0);
        assert_near(vp.vel(3.0), 2.0);
        assert_near(vp.acc(3.0), -1.0);
    }
}
