// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! State and motion validity, kept as two separate traits.
//!
//! Upstream OMPL's `StateValidityChecker` also owns a `MotionValidator`
//! pointer, so a space's "is this state okay" object and its "is this
//! segment okay" object are coupled through the same interface. Keeping them
//! separate here means a planner takes both explicitly and cannot forget
//! one: [`crate::rrt_connect::rrt_connect`] checks a candidate state's
//! validity and the motion into it as two distinct steps, and the type
//! signature makes that visible at the call site instead of hidden inside a
//! single checker's internals.

use crate::space::StateSpace;

/// Answers whether a single state is free of obstacles or constraint
/// violations.
pub trait StateValidityChecker<S: StateSpace> {
    /// Whether `state` may appear on a path.
    fn is_valid(&self, state: &S::State) -> bool;
}

impl<S, F> StateValidityChecker<S> for F
where
    S: StateSpace,
    F: Fn(&S::State) -> bool,
{
    fn is_valid(&self, state: &S::State) -> bool {
        self(state)
    }
}

/// Answers whether the motion from `from` to `to` (as traced by
/// [`StateSpace::interpolate`]) is free of obstacles or constraint
/// violations.
///
/// The endpoints themselves are not implied valid by a `true` result: a
/// caller checks `from` and `to` with a [`StateValidityChecker`] on its own.
/// That split is what lets one [`DiscreteMotionValidator`] be reused against
/// any checker without either side silently skipping the other's job.
pub trait MotionValidator<S: StateSpace> {
    /// Whether the open segment from `from` to `to` is valid.
    fn is_motion_valid(&self, space: &S, from: &S::State, to: &S::State) -> bool;
}

/// Checks a motion by recursive bisection over a fixed number of interior
/// sample points.
///
/// The segment is divided into `ceil(distance / resolution)` equal steps.
/// Rather than scanning front-to-back, the *midpoint* is checked first, then
/// each half is checked recursively (midpoint of the half, then quarters,
/// ...). A front-to-back scan costs `O(n)` checks even when the very first
/// interior point is invalid; bisection finds that same failure in one
/// check, and costs no more than `O(n)` in the worst case (an entirely valid
/// motion, where every point still has to be visited).
pub struct DiscreteMotionValidator<'c, S, C> {
    checker: &'c C,
    resolution: f64,
    _space: std::marker::PhantomData<S>,
}

impl<'c, S, C> DiscreteMotionValidator<'c, S, C>
where
    S: StateSpace,
    C: StateValidityChecker<S>,
{
    /// `resolution` is the maximum [`StateSpace::distance`] spacing between
    /// two consecutively checked interior states: a motion no longer than
    /// `resolution` is accepted without sampling its interior at all.
    ///
    /// # Panics
    /// If `resolution` is not finite and positive.
    pub fn new(checker: &'c C, resolution: f64) -> Self {
        assert!(
            resolution.is_finite() && resolution > 0.0,
            "DiscreteMotionValidator resolution must be finite and positive, got {resolution}"
        );
        Self {
            checker,
            resolution,
            _space: std::marker::PhantomData,
        }
    }

    /// Checks sample index `mid = first + (last - first) / 2` first, then
    /// recurses on `[first, mid - 1]` and `[mid + 1, last]`. `total` is the
    /// step count the segment was divided into, used to convert a sample
    /// index into an interpolation parameter `t = index / total`.
    fn check_range(
        &self,
        space: &S,
        from: &S::State,
        to: &S::State,
        first: u64,
        last: u64,
        total: u64,
    ) -> bool {
        if first > last {
            return true;
        }
        let mid = first + (last - first) / 2;
        let t = mid as f64 / total as f64;
        let state = space.interpolate(from, to, t);
        if !self.checker.is_valid(&state) {
            return false;
        }
        (mid == first || self.check_range(space, from, to, first, mid - 1, total))
            && (mid == last || self.check_range(space, from, to, mid + 1, last, total))
    }
}

impl<S, C> MotionValidator<S> for DiscreteMotionValidator<'_, S, C>
where
    S: StateSpace,
    C: StateValidityChecker<S>,
{
    fn is_motion_valid(&self, space: &S, from: &S::State, to: &S::State) -> bool {
        let dist = space.distance(from, to);
        if dist <= self.resolution {
            return true;
        }
        let steps = (dist / self.resolution).ceil() as u64;
        // `steps >= 2` here: `dist > resolution` was just established, and
        // `ceil` of anything greater than 1 is at least 2, so interior
        // indices `1 ..= steps - 1` are always non-empty.
        self.check_range(space, from, to, 1, steps - 1, steps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::space::RealVectorSpace;

    fn space() -> RealVectorSpace {
        RealVectorSpace::new(vec![(-10.0, 10.0)]).unwrap()
    }

    #[test]
    fn short_motion_within_resolution_is_valid_without_sampling() {
        let s = space();
        let never = |_: &Vec<f64>| false;
        let mv = DiscreteMotionValidator::new(&never, 1.0);
        assert!(mv.is_motion_valid(&s, &vec![0.0], &vec![0.5]));
    }

    #[test]
    fn all_valid_interior_is_valid() {
        let s = space();
        let always = |_: &Vec<f64>| true;
        let mv = DiscreteMotionValidator::new(&always, 0.1);
        assert!(mv.is_motion_valid(&s, &vec![0.0], &vec![5.0]));
    }

    #[test]
    fn invalid_midpoint_is_caught() {
        let s = space();
        // Invalid only at the exact midpoint of [0, 10] with resolution 1 (10 steps): x == 5.0.
        let checker = |state: &Vec<f64>| (state[0] - 5.0).abs() > 1e-9;
        let mv = DiscreteMotionValidator::new(&checker, 1.0);
        assert!(!mv.is_motion_valid(&s, &vec![0.0], &vec![10.0]));
    }

    #[test]
    fn invalid_near_far_endpoint_is_still_caught() {
        let s = space();
        // Invalid only just before the far end: bisection must recurse into
        // the second half to find it, not stop after checking the midpoint.
        let checker = |state: &Vec<f64>| (state[0] - 9.0).abs() > 1e-9;
        let mv = DiscreteMotionValidator::new(&checker, 1.0);
        assert!(!mv.is_motion_valid(&s, &vec![0.0], &vec![10.0]));
    }

    #[test]
    fn closure_checker_works_directly_as_state_validity_checker() {
        let s = space();
        let checker = |state: &Vec<f64>| state[0] >= 0.0;
        assert!(StateValidityChecker::<RealVectorSpace>::is_valid(
            &checker,
            &vec![1.0]
        ));
        assert!(!StateValidityChecker::<RealVectorSpace>::is_valid(
            &checker,
            &vec![-1.0]
        ));
        let _ = s;
    }
}
