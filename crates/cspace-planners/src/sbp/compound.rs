// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! A heterogeneous product of subspaces: what a MoveIt `JointModelGroup`
//! actually is once it mixes joint types -- an arm's revolute joints next to
//! a mobile base's floating pose, say -- rather than looking like any single
//! [`StateSpace`] impl this crate ships on its own.

use rand::Rng;

use crate::sbp::error::SbpError;
use crate::sbp::sampling::sample_simplex;
use crate::sbp::se3::{Se3Space, Se3State};
use crate::sbp::so2::So2Space;
use crate::sbp::space::{RealVectorSpace, StateSpace};

/// One subspace's value inside a [`CompoundSpace`]'s state.
///
/// [`StateSpace`] requires a single fixed `State` associated type, but a
/// compound space holds subspaces of genuinely different state types side
/// by side: a `Vec<f64>` for a block of prismatic joints, an `f64` for one
/// continuous joint's angle, an [`Se3State`] for a floating joint. This enum
/// is the common type that makes storing them together possible; each
/// concrete subspace is wrapped in a private adapter (below) that speaks
/// only `CompoundValue` at the [`StateSpace`] boundary and unwraps to its
/// own native type internally, panicking if a state ever carries the wrong
/// variant for its subspace -- which would mean [`CompoundSpace`] itself
/// built a mismatched state, not something a caller can trigger through the
/// public API.
#[derive(Debug, Clone, PartialEq)]
pub enum CompoundValue {
    /// A [`RealVectorSpace`] subspace's value.
    RealVector(Vec<f64>),
    /// A [`So2Space`] subspace's value.
    So2(f64),
    /// An [`Se3Space`] subspace's value.
    Se3(Se3State),
}

impl CompoundValue {
    fn as_real_vector(&self) -> &Vec<f64> {
        match self {
            CompoundValue::RealVector(v) => v,
            other => panic!("CompoundValue: expected RealVector, got {other:?}"),
        }
    }

    fn as_so2(&self) -> &f64 {
        match self {
            CompoundValue::So2(v) => v,
            other => panic!("CompoundValue: expected So2, got {other:?}"),
        }
    }

    fn as_se3(&self) -> &Se3State {
        match self {
            CompoundValue::Se3(v) => v,
            other => panic!("CompoundValue: expected Se3, got {other:?}"),
        }
    }
}

struct RealVectorAdapter(RealVectorSpace);

impl StateSpace for RealVectorAdapter {
    type State = CompoundValue;

    fn dimension(&self) -> usize {
        self.0.dimension()
    }

    fn distance(&self, a: &CompoundValue, b: &CompoundValue) -> f64 {
        self.0.distance(a.as_real_vector(), b.as_real_vector())
    }

    fn interpolate(&self, from: &CompoundValue, to: &CompoundValue, t: f64) -> CompoundValue {
        CompoundValue::RealVector(
            self.0
                .interpolate(from.as_real_vector(), to.as_real_vector(), t),
        )
    }

    fn enforce_bounds(&self, state: &mut CompoundValue) {
        let CompoundValue::RealVector(v) = state else {
            panic!("CompoundValue: expected RealVector, got {state:?}");
        };
        self.0.enforce_bounds(v);
    }

    fn satisfies_bounds(&self, state: &CompoundValue) -> bool {
        self.0.satisfies_bounds(state.as_real_vector())
    }

    fn sample_uniform(&self, rng: &mut dyn Rng) -> CompoundValue {
        CompoundValue::RealVector(self.0.sample_uniform(rng))
    }

    fn sample_near(&self, rng: &mut dyn Rng, center: &CompoundValue, radius: f64) -> CompoundValue {
        CompoundValue::RealVector(self.0.sample_near(rng, center.as_real_vector(), radius))
    }
}

struct So2Adapter(So2Space);

impl StateSpace for So2Adapter {
    type State = CompoundValue;

    fn dimension(&self) -> usize {
        self.0.dimension()
    }

    fn distance(&self, a: &CompoundValue, b: &CompoundValue) -> f64 {
        self.0.distance(a.as_so2(), b.as_so2())
    }

    fn interpolate(&self, from: &CompoundValue, to: &CompoundValue, t: f64) -> CompoundValue {
        CompoundValue::So2(self.0.interpolate(from.as_so2(), to.as_so2(), t))
    }

    fn enforce_bounds(&self, state: &mut CompoundValue) {
        let CompoundValue::So2(v) = state else {
            panic!("CompoundValue: expected So2, got {state:?}");
        };
        self.0.enforce_bounds(v);
    }

    fn satisfies_bounds(&self, state: &CompoundValue) -> bool {
        self.0.satisfies_bounds(state.as_so2())
    }

    fn sample_uniform(&self, rng: &mut dyn Rng) -> CompoundValue {
        CompoundValue::So2(self.0.sample_uniform(rng))
    }

    fn sample_near(&self, rng: &mut dyn Rng, center: &CompoundValue, radius: f64) -> CompoundValue {
        CompoundValue::So2(self.0.sample_near(rng, center.as_so2(), radius))
    }
}

struct Se3Adapter(Se3Space);

impl StateSpace for Se3Adapter {
    type State = CompoundValue;

    fn dimension(&self) -> usize {
        self.0.dimension()
    }

    fn distance(&self, a: &CompoundValue, b: &CompoundValue) -> f64 {
        self.0.distance(a.as_se3(), b.as_se3())
    }

    fn interpolate(&self, from: &CompoundValue, to: &CompoundValue, t: f64) -> CompoundValue {
        CompoundValue::Se3(self.0.interpolate(from.as_se3(), to.as_se3(), t))
    }

    fn enforce_bounds(&self, state: &mut CompoundValue) {
        let CompoundValue::Se3(v) = state else {
            panic!("CompoundValue: expected Se3, got {state:?}");
        };
        self.0.enforce_bounds(v);
    }

    fn satisfies_bounds(&self, state: &CompoundValue) -> bool {
        self.0.satisfies_bounds(state.as_se3())
    }

    fn sample_uniform(&self, rng: &mut dyn Rng) -> CompoundValue {
        CompoundValue::Se3(self.0.sample_uniform(rng))
    }

    fn sample_near(&self, rng: &mut dyn Rng, center: &CompoundValue, radius: f64) -> CompoundValue {
        CompoundValue::Se3(self.0.sample_near(rng, center.as_se3(), radius))
    }
}

/// A heterogeneous product of subspaces, each with its own weight in the
/// combined metric.
///
/// [`distance`](StateSpace::distance) is the weighted sum of each
/// subspace's own distance and [`State`](StateSpace::State) is one
/// [`CompoundValue`] per subspace, in the order they were added -- both
/// direct generalizations of [`Se3Space`]'s translation-plus-rotation case
/// to `N` subspaces of any of this crate's [`StateSpace`] kinds. Held as
/// `Box<dyn StateSpace<State = CompoundValue>>` because that vtable is
/// exactly what an earlier commit made [`StateSpace`] object-safe for: this
/// is the space that actually needs it, since a group's subspaces have no
/// single concrete type in common.
pub struct CompoundSpace {
    subspaces: Vec<(Box<dyn StateSpace<State = CompoundValue>>, f64)>,
}

impl CompoundSpace {
    /// Builds a compound space from subspaces and their weights in the
    /// combined metric, in order. Each subspace is a boxed adapter built by
    /// [`CompoundSpace::real_vector`], [`CompoundSpace::so2`], or
    /// [`CompoundSpace::se3`].
    ///
    /// # Errors
    /// [`SbpError::NoSubspaces`] if `subspaces` is empty.
    /// [`SbpError::InvalidSubspaceWeight`] if any weight is negative or
    /// non-finite.
    pub fn new(
        subspaces: Vec<(Box<dyn StateSpace<State = CompoundValue>>, f64)>,
    ) -> Result<Self, SbpError> {
        if subspaces.is_empty() {
            return Err(SbpError::NoSubspaces);
        }
        for (index, (_, weight)) in subspaces.iter().enumerate() {
            if !weight.is_finite() || *weight < 0.0 {
                return Err(SbpError::InvalidSubspaceWeight {
                    index,
                    value: *weight,
                });
            }
        }
        Ok(Self { subspaces })
    }

    /// Wraps a [`RealVectorSpace`] (a block of prismatic or bounded
    /// revolute joints) for use as a [`CompoundSpace`] subspace.
    pub fn real_vector(space: RealVectorSpace) -> Box<dyn StateSpace<State = CompoundValue>> {
        Box::new(RealVectorAdapter(space))
    }

    /// Wraps a [`So2Space`] (one continuous joint) for use as a
    /// [`CompoundSpace`] subspace.
    pub fn so2(space: So2Space) -> Box<dyn StateSpace<State = CompoundValue>> {
        Box::new(So2Adapter(space))
    }

    /// Wraps an [`Se3Space`] (one floating joint) for use as a
    /// [`CompoundSpace`] subspace.
    pub fn se3(space: Se3Space) -> Box<dyn StateSpace<State = CompoundValue>> {
        Box::new(Se3Adapter(space))
    }
}

impl StateSpace for CompoundSpace {
    type State = Vec<CompoundValue>;

    fn dimension(&self) -> usize {
        self.subspaces
            .iter()
            .map(|(space, _)| space.dimension())
            .sum()
    }

    fn distance(&self, a: &Vec<CompoundValue>, b: &Vec<CompoundValue>) -> f64 {
        debug_assert_eq!(a.len(), self.subspaces.len());
        debug_assert_eq!(b.len(), self.subspaces.len());
        self.subspaces
            .iter()
            .enumerate()
            .map(|(i, (space, weight))| weight * space.distance(&a[i], &b[i]))
            .sum()
    }

    fn interpolate(
        &self,
        from: &Vec<CompoundValue>,
        to: &Vec<CompoundValue>,
        t: f64,
    ) -> Vec<CompoundValue> {
        debug_assert_eq!(from.len(), self.subspaces.len());
        debug_assert_eq!(to.len(), self.subspaces.len());
        // Each subspace's own interpolate already returns `from[i]` /
        // `to[i]` back exactly at t <= 0.0 / t >= 1.0 (every StateSpace impl
        // in this crate guarantees that), so this composition inherits the
        // same exactness at the whole-state level for free -- no extra
        // short-circuit needed here.
        self.subspaces
            .iter()
            .enumerate()
            .map(|(i, (space, _))| space.interpolate(&from[i], &to[i], t))
            .collect()
    }

    fn enforce_bounds(&self, state: &mut Vec<CompoundValue>) {
        debug_assert_eq!(state.len(), self.subspaces.len());
        for (i, (space, _)) in self.subspaces.iter().enumerate() {
            space.enforce_bounds(&mut state[i]);
        }
    }

    fn satisfies_bounds(&self, state: &Vec<CompoundValue>) -> bool {
        debug_assert_eq!(state.len(), self.subspaces.len());
        self.subspaces
            .iter()
            .enumerate()
            .all(|(i, (space, _))| space.satisfies_bounds(&state[i]))
    }

    fn sample_uniform(&self, rng: &mut dyn Rng) -> Vec<CompoundValue> {
        self.subspaces
            .iter()
            .map(|(space, _)| space.sample_uniform(rng))
            .collect()
    }

    fn sample_near(
        &self,
        rng: &mut dyn Rng,
        center: &Vec<CompoundValue>,
        radius: f64,
    ) -> Vec<CompoundValue> {
        debug_assert_eq!(center.len(), self.subspaces.len());
        if radius <= 0.0 {
            let mut state = center.clone();
            self.enforce_bounds(&mut state);
            return state;
        }
        // distance is a weighted sum of N independent subspace metrics, the
        // N-subspace generalization of Se3Space's two-part split: a uniform
        // draw from the (N-1)-simplex gives each subspace a fraction of the
        // radius budget, and the fractions sum to 1 by construction, so the
        // weighted sum of per-subspace distances (each within its own
        // budget) never exceeds `radius` regardless of how the draw falls.
        let fractions = sample_simplex(rng, self.subspaces.len());
        self.subspaces
            .iter()
            .enumerate()
            .map(|(i, (space, weight))| {
                let budget = fractions[i] * radius;
                if *weight <= 0.0 {
                    // A weight of 0 means this subspace contributes nothing
                    // to distance, so any value is within budget; sample
                    // freely instead of dividing by zero.
                    space.sample_uniform(rng)
                } else {
                    space.sample_near(rng, &center[i], budget / weight)
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::sbp::test_support::{
        assert_metric_and_interpolation_axioms, assert_sample_near_stays_within_radius,
    };
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    fn space() -> CompoundSpace {
        CompoundSpace::new(vec![
            (
                CompoundSpace::real_vector(
                    RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap(),
                ),
                1.0,
            ),
            (CompoundSpace::so2(So2Space::new()), 1.0),
            (
                CompoundSpace::se3(
                    Se3Space::new([(-5.0, 5.0), (-5.0, 5.0), (-5.0, 5.0)], 1.0).unwrap(),
                ),
                0.5,
            ),
        ])
        .unwrap()
    }

    #[test]
    fn no_subspaces_is_rejected() {
        // Not assert_eq!/unwrap_err!: CompoundSpace holds
        // Box<dyn StateSpace<..>> and so cannot implement Debug itself,
        // which both of those require even on the Ok side of a Result.
        match CompoundSpace::new(vec![]) {
            Err(e) => assert_eq!(e, SbpError::NoSubspaces),
            Ok(_) => panic!("expected Err(NoSubspaces)"),
        }
    }

    #[test]
    fn negative_subspace_weight_is_rejected() {
        match CompoundSpace::new(vec![(CompoundSpace::so2(So2Space::new()), -1.0)]) {
            Err(e) => assert_eq!(
                e,
                SbpError::InvalidSubspaceWeight {
                    index: 0,
                    value: -1.0
                }
            ),
            Ok(_) => panic!("expected Err(InvalidSubspaceWeight)"),
        }
    }

    #[test]
    fn dimension_is_the_sum_of_subspace_dimensions() {
        // RealVectorSpace(2) + So2Space(1) + Se3Space(6) = 9.
        assert_eq!(space().dimension(), 9);
    }

    #[test]
    fn interpolate_endpoints_are_exact() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let a = s.sample_uniform(&mut rng);
        let b = s.sample_uniform(&mut rng);
        assert_eq!(s.interpolate(&a, &b, 0.0), a);
        assert_eq!(s.interpolate(&a, &b, 1.0), b);
    }

    #[test]
    fn sample_uniform_satisfies_bounds() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(2);
        for _ in 0..500 {
            let state = s.sample_uniform(&mut rng);
            assert!(s.satisfies_bounds(&state));
        }
    }

    #[test]
    fn metric_and_interpolation_axioms_hold() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        assert_metric_and_interpolation_axioms(
            &s,
            &mut rng,
            |rng| s.sample_uniform(rng),
            2000,
            1e-6,
        );
    }

    #[test]
    fn sample_near_stays_within_radius() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(4);
        for &radius in &[0.1, 1.0, 5.0] {
            let center = s.sample_uniform(&mut rng);
            assert_sample_near_stays_within_radius(&s, &mut rng, &center, radius, 500, 1e-9);
        }
    }

    #[test]
    fn zero_weight_subspace_does_not_consume_radius_budget() {
        let s = CompoundSpace::new(vec![
            (
                CompoundSpace::real_vector(RealVectorSpace::new(vec![(-10.0, 10.0)]).unwrap()),
                1.0,
            ),
            (CompoundSpace::so2(So2Space::new()), 0.0),
        ])
        .unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(5);
        let center = s.sample_uniform(&mut rng);
        assert_sample_near_stays_within_radius(&s, &mut rng, &center, 0.5, 500, 1e-9);
    }

    /// `distance`'s weighted-sum structure -- `sum(weight_i * distance_i)`
    /// -- mirrors upstream `JointModelGroup::distance` (`joint_model_group.cpp:462-472`,
    /// `d += factor_i * joint_i->distance(...)`), though the weight values
    /// themselves are this crate's own extent-normalization rule rather than
    /// upstream's `getDistanceFactor()` (an already-documented deviation --
    /// see `JointModelGroupSpace`'s doc comment). What is checked here is
    /// the arithmetic itself: two subspaces with independently-known
    /// per-subspace distances (`RealVectorSpace`'s `|3.0| = 3.0`,
    /// `So2Space`'s shorter arc over `PI/3`) at deliberately unequal
    /// weights, summed by hand outside `CompoundSpace::distance` and
    /// compared against its actual output -- not a value this
    /// implementation itself produced.
    #[test]
    fn distance_is_the_hand_computed_weighted_sum_at_a_known_value() {
        let s = CompoundSpace::new(vec![
            (
                CompoundSpace::real_vector(RealVectorSpace::new(vec![(-10.0, 10.0)]).unwrap()),
                2.0,
            ),
            (CompoundSpace::so2(So2Space::new()), 5.0),
        ])
        .unwrap();
        let a = vec![
            CompoundValue::RealVector(vec![0.0]),
            CompoundValue::So2(0.0),
        ];
        let b = vec![
            CompoundValue::RealVector(vec![3.0]),
            CompoundValue::So2(PI / 3.0),
        ];
        let expected = 2.0 * 3.0 + 5.0 * (PI / 3.0);
        let actual = s.distance(&a, &b);
        assert!(
            (actual - expected).abs() < 1e-9,
            "distance = {actual}, hand-computed weighted sum = {expected}"
        );
    }

    // PORTING-PLAN.md:1269: whether the StateSpace trait carries a
    // heterogeneous product's distance correctly — specifically, whether
    // summing per-subspace distances of genuinely different units (metres,
    // radians) through one weight each still produces a metric — has "so
    // far been a comment's assertion, never verified." `distance`'s own doc
    // comment argument ("a nonnegative weighted sum of metrics satisfies the
    // triangle inequality termwise") is a real theorem, not hand-waving, but
    // this is a boundary check of it at a deliberately extreme, constructed
    // weight ratio rather than the `1.0`/`1.0`/`0.5` weights
    // `metric_and_interpolation_axioms_hold` above happens to use.

    /// A metres-weighted-1000x-more-than-radians space, with one of the two
    /// constructed points straddling the `So2` seam: if summing differently
    /// scaled per-subspace distances into one scalar could ever break the
    /// triangle inequality, an extreme weight ratio combined with a seam
    /// crossing is where it would show up, not at comparable weights on an
    /// interior point.
    #[test]
    fn weighted_distance_across_mixed_units_holds_the_triangle_inequality_at_an_extreme_ratio() {
        let s = CompoundSpace::new(vec![
            (
                CompoundSpace::real_vector(RealVectorSpace::new(vec![(-1000.0, 1000.0)]).unwrap()),
                1000.0,
            ),
            (CompoundSpace::so2(So2Space::new()), 0.001),
        ])
        .unwrap();

        let a = vec![
            CompoundValue::RealVector(vec![-500.0]),
            CompoundValue::So2(-PI + 0.05),
        ];
        let b = vec![
            CompoundValue::RealVector(vec![500.0]),
            CompoundValue::So2(PI - 0.05),
        ];
        let c = vec![
            CompoundValue::RealVector(vec![0.0]),
            CompoundValue::So2(0.0),
        ];

        let d_ab = s.distance(&a, &b);
        let d_ba = s.distance(&b, &a);
        assert!(
            (d_ab - d_ba).abs() < 1e-9,
            "distance not symmetric under mixed-unit weighting: {d_ab} vs {d_ba}"
        );

        let d_ac = s.distance(&a, &c);
        let d_bc = s.distance(&b, &c);
        assert!(
            d_ac <= d_ab + d_bc + 1e-9,
            "triangle inequality violated under mixed-unit weighting: distance(a,c) = {d_ac} > \
             distance(a,b) + distance(b,c) = {}",
            d_ab + d_bc
        );

        // Zero exactly on the diagonal, regardless of how extreme the
        // per-subspace weights are.
        assert_eq!(s.distance(&a, &a), 0.0);
    }
}
