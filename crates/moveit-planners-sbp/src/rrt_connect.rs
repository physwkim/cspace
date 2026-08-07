// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Bidirectional RRT-Connect (Kuffner & LaValle, "RRT-Connect: An Efficient
//! Approach to Single-Query Path Planning", ICRA 2000).
//!
//! Two trees grow, one rooted at the start state and one at the goal. Each
//! iteration, one tree takes a single step (`extend`) toward a sampled
//! state; if that step succeeds, the *other* tree greedily walks
//! (`connect`) all the way toward wherever the first tree just grew, one
//! step at a time, until it either reaches that exact state (the trees have
//! met — a path exists) or gets blocked. The trees swap roles every
//! iteration.
//!
//! There is no upstream C++ implementation to port here and no oracle to
//! check answers against (PORTING-PLAN.md §2, §6.3: no Rust OMPL
//! equivalent exists). Correctness in this module is established by the
//! property tests below, not by comparison to a reference.

use std::time::{Duration, Instant};

use rand::{Rng, RngExt};

use crate::nn::Gnat;
use crate::space::StateSpace;
use crate::validity::{MotionValidator, StateValidityChecker};

/// The condition(s) that stop [`rrt_connect`] from growing its trees
/// further.
///
/// This is a sum type rather than "an iteration cap plus an optional
/// deadline" so that a caller who needs the [determinism
/// guarantee](rrt_connect#determinism) can select [`Termination::Iterations`]
/// and get it *by construction*: that variant carries no [`Duration`], so
/// [`rrt_connect`]'s loop has nothing to call [`Instant::now`] against and
/// cannot become clock-bound even by accident. [`Termination::Deadline`] and
/// [`Termination::Both`] are the caller's explicit choice to trade that
/// guarantee for a wall-clock safety net.
#[derive(Debug, Clone, Copy)]
pub enum Termination {
    /// Run for at most `max_iterations` iterations (one `extend` plus, if it
    /// succeeds, one `connect` attempt each). Never reads the wall clock:
    /// two calls with an identical seed always run exactly the same number
    /// of iterations.
    Iterations(usize),
    /// Run until `deadline` elapses. Not deterministic: the number of
    /// iterations completed before the deadline fires depends on machine
    /// speed and load.
    Deadline(Duration),
    /// Run until either bound is hit, whichever comes first. Not
    /// deterministic, for the same reason as [`Termination::Deadline`].
    Both {
        /// See [`Termination::Iterations`].
        max_iterations: usize,
        /// See [`Termination::Deadline`].
        deadline: Duration,
    },
}

/// Why [`rrt_connect`] returned without a path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum PlanningFailure {
    /// `start` or `goal` itself failed [`StateValidityChecker::is_valid`]:
    /// an invalid endpoint can never appear on a valid path, so nothing was
    /// searched for.
    #[error("start or goal state is itself invalid")]
    InvalidEndpoint,
    /// The `max_iterations` bound in `params.termination` was reached
    /// before a solution was found.
    #[error("no path found within the iteration budget")]
    IterationsExhausted,
    /// The `deadline` bound in `params.termination` elapsed before a
    /// solution was found.
    #[error("no path found before the deadline")]
    DeadlineExhausted,
}

/// Tuning parameters for [`rrt_connect`].
#[derive(Debug, Clone)]
pub struct RrtConnectParams {
    /// Maximum distance (in [`StateSpace::distance`] units) a single
    /// `extend` step advances a tree toward its target.
    pub step_size: f64,
    /// Probability in `[0.0, 1.0]` that a tree's growth target for a given
    /// iteration is the *other* tree's root — the goal, when growing the
    /// start-rooted tree; the start, when growing the goal-rooted tree —
    /// instead of a uniform random sample. Biases both trees to reach
    /// toward each other directly rather than relying solely on `connect`'s
    /// own greedy walk to close the gap.
    pub goal_bias: f64,
    /// When to stop growing the trees. See [`Termination`].
    pub termination: Termination,
    /// Nearest-neighbour index branching factor, forwarded to
    /// [`Gnat::new`] for each tree.
    pub nn_degree: usize,
}

impl RrtConnectParams {
    fn assert_valid(&self) {
        assert!(
            self.step_size.is_finite() && self.step_size > 0.0,
            "RrtConnectParams::step_size must be finite and positive, got {}",
            self.step_size
        );
        assert!(
            (0.0..=1.0).contains(&self.goal_bias),
            "RrtConnectParams::goal_bias must be within [0.0, 1.0], got {}",
            self.goal_bias
        );
        assert!(
            self.nn_degree > 0,
            "RrtConnectParams::nn_degree must be at least 1, got 0"
        );
    }
}

/// A source of constraint-satisfying candidate states for [`rrt_connect`]'s
/// uniform-sampling step.
///
/// Mirrors upstream's `ompl_interface::ConstrainedSampler`
/// (`moveit_planners/ompl/ompl_interface/{include,src}/moveit/ompl_interface/detail/constrained_sampler.{hpp,cpp}`),
/// which wraps a `constraint_samplers::ConstraintSamplerPtr` and is
/// installed as the OMPL state space's sampler allocator whenever a
/// planning request carries path constraints
/// (`model_based_planning_context.cpp`'s `configure()` /
/// `allocPathConstrainedSampler()`) — it intercepts every uniform sample
/// OMPL's planners draw during tree growth, not just goal sampling. This
/// trait is the same seam on this port's side: [`rrt_connect`]'s uniform
/// branch calls it instead of [`StateSpace::sample_uniform`] directly.
///
/// `try_sample` returning `None` is an ordinary "this attempt found
/// nothing" outcome (the constraint sampler's own retry budget was
/// exhausted), not an error — [`Sampler`]'s uniform-sampling step retries up
/// to three times before falling back to [`StateSpace::sample_uniform`], the
/// same reaction described in [`Sampler`]'s own doc comment.
pub trait ConstrainedStateSampler<S: StateSpace> {
    /// Attempts to draw one constraint-satisfying state. `None` means this
    /// attempt failed; the caller may retry or fall back.
    fn try_sample(&self, rng: &mut dyn Rng) -> Option<S::State>;
}

/// [`rrt_connect`]'s source of randomness, bundled with an optional
/// constraint-driven override for its uniform-sampling step.
///
/// Mirrors upstream's `ConstrainedSampler` itself wrapping a plain
/// `default_` sampler (`constrained_sampler.hpp`'s `default_` field) rather
/// than living beside it as an unrelated argument — see
/// [`ConstrainedStateSampler`]'s doc comment for the fuller citation. `rng`
/// and `constrained_sampler` are grouped into one type, rather than two
/// separate [`rrt_connect`] parameters, for the same reason: they jointly
/// answer one question, "what state do we get when this iteration asks for
/// a sample," and grouping keeps [`rrt_connect`]'s own parameter list from
/// growing by one argument per sampling concern it grows in the future.
pub struct Sampler<'a, S: StateSpace, R: Rng> {
    /// The plain RNG. Used directly for [`RrtConnectParams::goal_bias`]'s
    /// coin flip regardless of `constrained_sampler`, and forwarded to it
    /// (or to [`StateSpace::sample_uniform`]) for the uniform-sampling
    /// branch.
    pub rng: &'a mut R,
    /// See [`ConstrainedStateSampler`]. `None` means every uniform sample
    /// comes from `rng` via [`StateSpace::sample_uniform`] directly — the
    /// only behaviour that existed before this type did.
    pub constrained_sampler: Option<&'a dyn ConstrainedStateSampler<S>>,
}

impl<'a, S: StateSpace, R: Rng> Sampler<'a, S, R> {
    /// A [`Sampler`] with no constraint-driven override: every uniform
    /// sample comes straight from `rng`.
    pub fn unconstrained(rng: &'a mut R) -> Self {
        Self {
            rng,
            constrained_sampler: None,
        }
    }

    /// Draws a uniform sample for [`rrt_connect`]'s growth step, preferring
    /// `constrained_sampler` when one is present.
    ///
    /// Mirrors `ConstrainedSampler::sampleUniform`
    /// (`constrained_sampler.cpp:83-87`): up to three attempts at the
    /// constraint sampler (`sampleC`) before falling back to a plain
    /// uniform sample (upstream's `default_->sampleUniform`, this port's
    /// [`StateSpace::sample_uniform`]) — verbatim,
    /// `if (!sampleC(state) && !sampleC(state) && !sampleC(state))
    /// default_->sampleUniform(state);`.
    fn sample_uniform(&mut self, space: &S) -> S::State {
        if let Some(sampler) = self.constrained_sampler {
            for _ in 0..3 {
                if let Some(state) = sampler.try_sample(self.rng) {
                    return state;
                }
            }
        }
        space.sample_uniform(self.rng)
    }
}

struct TreeNode<P> {
    state: P,
    parent: Option<usize>,
}

struct Tree<S: StateSpace> {
    nodes: Vec<TreeNode<S::State>>,
    nn: Gnat<S, usize>,
}

impl<S: StateSpace> Tree<S> {
    fn new(space: &S, root: S::State, nn_degree: usize) -> Self {
        let mut nn = Gnat::new(nn_degree);
        nn.insert(space, root.clone(), 0);
        Self {
            nodes: vec![TreeNode {
                state: root,
                parent: None,
            }],
            nn,
        }
    }

    fn push(&mut self, space: &S, state: S::State, parent: usize) -> usize {
        let index = self.nodes.len();
        self.nn.insert(space, state.clone(), index);
        self.nodes.push(TreeNode {
            state,
            parent: Some(parent),
        });
        index
    }

    /// The path from this tree's root to `index`, root first.
    fn path_to_root(&self, mut index: usize) -> Vec<S::State> {
        let mut reversed = Vec::new();
        loop {
            reversed.push(self.nodes[index].state.clone());
            match self.nodes[index].parent {
                Some(parent) => index = parent,
                None => break,
            }
        }
        reversed.reverse();
        reversed
    }
}

enum ExtendResult {
    /// No valid step could be taken: the nearest state's single step toward
    /// the target failed state or motion validity.
    Trapped,
    /// A valid step was taken but did not reach the target.
    Advanced(usize),
    /// A valid step was taken and landed exactly on the target.
    Reached(usize),
}

/// Takes a single bounded step from `tree`'s nearest node toward `target`.
fn extend<S, C, M>(
    space: &S,
    checker: &C,
    motion_validator: &M,
    tree: &mut Tree<S>,
    target: &S::State,
    step_size: f64,
) -> ExtendResult
where
    S: StateSpace,
    C: StateValidityChecker<S>,
    M: MotionValidator<S>,
{
    let (nearest_state, &nearest_index) = tree
        .nn
        .nearest(space, target)
        .expect("a tree always has at least its root");
    let nearest_state = nearest_state.clone();

    let dist = space.distance(&nearest_state, target);
    let reaches = dist <= step_size;
    let new_state = if reaches {
        // Always the literal `target`, never `nearest_state`, even when
        // `dist == 0.0`: a space with a degenerate metric (`So2Space`
        // wraparound, or a zero-weight `CompoundSpace` subspace) can have
        // `distance(a, b) == 0.0` for `a != b`. Short-circuiting on
        // `dist == 0.0` to return `nearest_index` without ever validating or
        // storing `target` breaks this function's own "`path.last() ==
        // Some(&goal)` exactly" contract, and can splice an unvalidated
        // edge into the returned path (the literal `target` is never
        // checked against `checker`/`motion_validator` at all in that
        // case). Folding `dist == 0.0` into the ordinary `reaches` branch
        // below makes every accepted step -- including a zero-distance one
        // -- go through the same validity checks and land the same literal
        // state.
        target.clone()
    } else {
        space.interpolate(&nearest_state, target, step_size / dist)
    };

    if !checker.is_valid(&new_state)
        || !motion_validator.is_motion_valid(space, &nearest_state, &new_state)
    {
        return ExtendResult::Trapped;
    }

    let index = tree.push(space, new_state, nearest_index);
    if reaches {
        ExtendResult::Reached(index)
    } else {
        ExtendResult::Advanced(index)
    }
}

/// Repeatedly `extend`s `tree` toward `target` until it is `Reached` or
/// `Trapped` — the "greedy connect loop".
fn connect<S, C, M>(
    space: &S,
    checker: &C,
    motion_validator: &M,
    tree: &mut Tree<S>,
    target: &S::State,
    step_size: f64,
) -> ExtendResult
where
    S: StateSpace,
    C: StateValidityChecker<S>,
    M: MotionValidator<S>,
{
    loop {
        match extend(space, checker, motion_validator, tree, target, step_size) {
            ExtendResult::Advanced(_) => continue,
            terminal => return terminal,
        }
    }
}

/// Plans a path from `start` to `goal` with bidirectional RRT-Connect.
///
/// Returns `Ok(path)` on success, with `path[0] == start` and
/// `path.last() == Some(&goal)` exactly, and every consecutive pair valid
/// under `motion_validator` (both properties are asserted directly by this
/// crate's tests, not just inferred). Returns `Err(PlanningFailure::InvalidEndpoint)`
/// immediately if `start` or `goal` is itself invalid, or
/// `Err(PlanningFailure::IterationsExhausted)` /
/// `Err(PlanningFailure::DeadlineExhausted)` if the corresponding bound in
/// `params.termination` is hit first — see [`PlanningFailure`].
///
/// # Determinism
/// Two calls with equivalent `space`/`checker`/`motion_validator` and an
/// identically-seeded `sampler.rng` produce byte-identical results when
/// `params.termination` is [`Termination::Iterations`] — that variant never
/// reads the wall clock, so nothing in this function can make the result
/// depend on machine speed. [`Termination::Deadline`] and
/// [`Termination::Both`] intentionally give up this guarantee; do not use
/// them where reproducibility matters. `sampler.constrained_sampler` being
/// `None` costs this guarantee nothing: every random draw still comes from
/// `sampler.rng` exactly as before. A `Some` constrained sampler is only as
/// deterministic as its own `try_sample` — this function does not add or
/// remove any determinism property from whatever was passed in.
///
/// # Panics
/// If `params` is out of range: see [`RrtConnectParams`]'s field docs.
pub fn rrt_connect<S, C, M, R>(
    space: &S,
    checker: &C,
    motion_validator: &M,
    start: S::State,
    goal: S::State,
    mut sampler: Sampler<'_, S, R>,
    params: &RrtConnectParams,
) -> Result<Vec<S::State>, PlanningFailure>
where
    S: StateSpace,
    C: StateValidityChecker<S>,
    M: MotionValidator<S>,
    R: Rng,
{
    params.assert_valid();

    if !checker.is_valid(&start) || !checker.is_valid(&goal) {
        return Err(PlanningFailure::InvalidEndpoint);
    }

    // `deadline` is `None` for `Termination::Iterations`: that is what makes
    // "never reads the wall clock" true by construction rather than by
    // convention — there is simply nothing here to check `Instant::now`
    // against.
    let (max_iterations, deadline) = match params.termination {
        Termination::Iterations(max_iterations) => (max_iterations, None),
        Termination::Deadline(deadline) => (usize::MAX, Some(Instant::now() + deadline)),
        Termination::Both {
            max_iterations,
            deadline,
        } => (max_iterations, Some(Instant::now() + deadline)),
    };

    let mut tree_a = Tree::new(space, start.clone(), params.nn_degree);
    let mut tree_b = Tree::new(space, goal.clone(), params.nn_degree);
    // Whether `tree_a` is currently the start-rooted tree (as opposed to the
    // goal-rooted one) — the two trees are swapped every iteration, and this
    // flag tracks which is which so the final path can be oriented
    // start-to-goal regardless of which tree closed the gap.
    let mut a_is_start = true;

    for _ in 0..max_iterations {
        if let Some(deadline) = deadline {
            if Instant::now() >= deadline {
                return Err(PlanningFailure::DeadlineExhausted);
            }
        }

        let other_root = if a_is_start { &goal } else { &start };
        let sample = if sampler.rng.random_bool(params.goal_bias) {
            other_root.clone()
        } else {
            sampler.sample_uniform(space)
        };

        let grown = match extend(
            space,
            checker,
            motion_validator,
            &mut tree_a,
            &sample,
            params.step_size,
        ) {
            ExtendResult::Trapped => None,
            ExtendResult::Advanced(index) | ExtendResult::Reached(index) => Some(index),
        };

        if let Some(index) = grown {
            let reached_state = tree_a.nodes[index].state.clone();
            if let ExtendResult::Reached(other_index) = connect(
                space,
                checker,
                motion_validator,
                &mut tree_b,
                &reached_state,
                params.step_size,
            ) {
                let mut path = tree_a.path_to_root(index);
                let mut other_path = tree_b.path_to_root(other_index);
                other_path.reverse();
                other_path.remove(0); // duplicate of `reached_state`, the meeting point
                path.extend(other_path);
                if !a_is_start {
                    path.reverse();
                }
                return Ok(path);
            }
        }

        std::mem::swap(&mut tree_a, &mut tree_b);
        a_is_start = !a_is_start;
    }

    Err(PlanningFailure::IterationsExhausted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compound::{CompoundSpace, CompoundValue};
    use crate::se3::{Se3Space, Se3State};
    use crate::so2::So2Space;
    use crate::space::RealVectorSpace;
    use crate::validity::DiscreteMotionValidator;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    fn params() -> RrtConnectParams {
        RrtConnectParams {
            step_size: 0.5,
            goal_bias: 0.05,
            termination: Termination::Iterations(10_000),
            nn_degree: 8,
        }
    }

    fn rng(seed: u64) -> ChaCha8Rng {
        ChaCha8Rng::seed_from_u64(seed)
    }

    #[test]
    #[should_panic(expected = "step_size")]
    fn zero_step_size_panics() {
        let mut p = params();
        p.step_size = 0.0;
        p.assert_valid();
    }

    #[test]
    #[should_panic(expected = "goal_bias")]
    fn out_of_range_goal_bias_panics() {
        let mut p = params();
        p.goal_bias = 1.5;
        p.assert_valid();
    }

    #[test]
    #[should_panic(expected = "nn_degree")]
    fn zero_nn_degree_panics() {
        let mut p = params();
        p.nn_degree = 0;
        p.assert_valid();
    }

    #[test]
    fn empty_space_reaches_start_and_goal_exactly() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap();
        let always_valid = |_: &Vec<f64>| true;
        let mv = DiscreteMotionValidator::new(&always_valid, 0.1);
        let start = vec![-5.0, -5.0];
        let goal = vec![5.0, 5.0];

        let path = rrt_connect(
            &space,
            &always_valid,
            &mv,
            start.clone(),
            goal.clone(),
            Sampler::unconstrained(&mut rng(1)),
            &params(),
        )
        .expect("open space must be solvable");

        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
    }

    #[test]
    fn wraparound_endpoints_with_zero_distance_but_unequal_values_reach_the_literal_goal() {
        // `So2Space::distance(-PI, PI) == 0.0` (they name the same point on
        // the circle) even though `-PI != PI` as `f64`. `extend`'s "dist ==
        // 0.0 -> already at target" shortcut cannot distinguish this from
        // the ordinary case where the nearest node is bit-identical to the
        // target, so it must not treat zero distance as "nothing to do".
        let space = So2Space::new();
        let always_valid = |_: &f64| true;
        let mv = DiscreteMotionValidator::new(&always_valid, 0.1);
        let start = -std::f64::consts::PI;
        let goal = std::f64::consts::PI;
        assert_eq!(
            space.distance(&start, &goal),
            0.0,
            "test setup: this only exercises the bug if distance is exactly zero"
        );

        let mut p = params();
        p.goal_bias = 1.0; // force the very first sample to be the other tree's root

        let path = rrt_connect(
            &space,
            &always_valid,
            &mv,
            start,
            goal,
            Sampler::unconstrained(&mut rng(1)),
            &p,
        )
        .expect("a fully open SO(2) space must be solvable");

        assert_eq!(
            path.last(),
            Some(&goal),
            "path did not end at the literal goal state: {path:?}"
        );
    }

    #[test]
    fn every_consecutive_pair_is_motion_valid() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap();
        // A vertical wall at x in [-1, 1] with a gap for y in [3, 4].
        let checker = |state: &Vec<f64>| {
            let (x, y) = (state[0], state[1]);
            !((-1.0..=1.0).contains(&x) && !(3.0..=4.0).contains(&y))
        };
        let mv = DiscreteMotionValidator::new(&checker, 0.05);
        let start = vec![-5.0, 0.0];
        let goal = vec![5.0, 0.0];

        let path = rrt_connect(
            &space,
            &checker,
            &mv,
            start,
            goal,
            Sampler::unconstrained(&mut rng(2)),
            &params(),
        )
        .expect("gap must be findable");

        assert!(path.len() >= 2);
        for pair in path.windows(2) {
            assert!(
                mv.is_motion_valid(&space, &pair[0], &pair[1]),
                "invalid segment {:?} -> {:?}",
                pair[0],
                pair[1]
            );
        }
    }

    #[test]
    fn narrow_gap_is_crossed() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap();
        let checker = |state: &Vec<f64>| {
            let (x, y) = (state[0], state[1]);
            !((-1.0..=1.0).contains(&x) && !(3.0..=4.0).contains(&y))
        };
        let mv = DiscreteMotionValidator::new(&checker, 0.05);
        let start = vec![-5.0, 0.0];
        let goal = vec![5.0, 0.0];

        let path = rrt_connect(
            &space,
            &checker,
            &mv,
            start,
            goal,
            Sampler::unconstrained(&mut rng(3)),
            &params(),
        )
        .expect("gap must be findable");

        // The wall spans x in [-1, 1]; the path must actually pass through
        // it, and only through the gap (y in [3, 4]) while doing so. It is
        // not enough that the planner merely reported success.
        assert!(
            path.iter().any(|s| (-1.0..=1.0).contains(&s[0])),
            "path never enters the wall's x range at all: {path:?}"
        );
        for state in path.iter().filter(|s| (-1.0..=1.0).contains(&s[0])) {
            assert!(
                (3.0..=4.0).contains(&state[1]),
                "path point {state:?} is inside the wall's x range but outside the gap"
            );
        }
    }

    #[test]
    fn closed_passage_fails_within_the_cap() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap();
        // Same wall, no gap: x in [-1, 1] is entirely blocked.
        let checker = |state: &Vec<f64>| !(-1.0..=1.0).contains(&state[0]);
        let mv = DiscreteMotionValidator::new(&checker, 0.05);
        let start = vec![-5.0, 0.0];
        let goal = vec![5.0, 0.0];

        let mut small_cap = params();
        small_cap.termination = Termination::Iterations(2_000);

        let result = rrt_connect(
            &space,
            &checker,
            &mv,
            start,
            goal,
            Sampler::unconstrained(&mut rng(4)),
            &small_cap,
        );
        assert_eq!(
            result,
            Err(PlanningFailure::IterationsExhausted),
            "a fully closed wall must not be crossed"
        );
    }

    #[test]
    fn deadline_exhausted_reports_correctly() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap();
        // Same fully-closed wall as `closed_passage_fails_within_the_cap`,
        // but bounded by a deadline rather than an iteration cap, to check
        // that `Termination::Deadline` is honoured and reported correctly.
        // This is the one test in this module that is allowed to depend on
        // wall-clock behaviour, since it is testing that behaviour directly.
        let checker = |state: &Vec<f64>| !(-1.0..=1.0).contains(&state[0]);
        let mv = DiscreteMotionValidator::new(&checker, 0.05);
        let start = vec![-5.0, 0.0];
        let goal = vec![5.0, 0.0];

        let mut short_deadline = params();
        short_deadline.termination = Termination::Deadline(Duration::from_millis(20));

        let result = rrt_connect(
            &space,
            &checker,
            &mv,
            start,
            goal,
            Sampler::unconstrained(&mut rng(4)),
            &short_deadline,
        );
        assert_eq!(result, Err(PlanningFailure::DeadlineExhausted));
    }

    #[test]
    fn invalid_start_fails_immediately() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0)]).unwrap();
        let checker = |state: &Vec<f64>| state[0] >= 0.0;
        let mv = DiscreteMotionValidator::new(&checker, 0.1);
        let result = rrt_connect(
            &space,
            &checker,
            &mv,
            vec![-1.0],
            vec![5.0],
            Sampler::unconstrained(&mut rng(5)),
            &params(),
        );
        assert_eq!(result, Err(PlanningFailure::InvalidEndpoint));
    }

    #[test]
    fn same_seed_gives_byte_identical_path() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap();
        let checker = |state: &Vec<f64>| {
            let (x, y) = (state[0], state[1]);
            !((-1.0..=1.0).contains(&x) && !(3.0..=4.0).contains(&y))
        };
        let mv = DiscreteMotionValidator::new(&checker, 0.05);
        let start = vec![-5.0, 0.0];
        let goal = vec![5.0, 0.0];

        let path_1 = rrt_connect(
            &space,
            &checker,
            &mv,
            start.clone(),
            goal.clone(),
            Sampler::unconstrained(&mut rng(42)),
            &params(),
        )
        .expect("gap must be findable");
        let path_2 = rrt_connect(
            &space,
            &checker,
            &mv,
            start,
            goal,
            Sampler::unconstrained(&mut rng(42)),
            &params(),
        )
        .expect("gap must be findable");

        assert_eq!(path_1, path_2);
    }

    #[test]
    fn different_seeds_give_different_paths() {
        let space = RealVectorSpace::new(vec![(-10.0, 10.0), (-10.0, 10.0)]).unwrap();
        let checker = |state: &Vec<f64>| {
            let (x, y) = (state[0], state[1]);
            !((-1.0..=1.0).contains(&x) && !(3.0..=4.0).contains(&y))
        };
        let mv = DiscreteMotionValidator::new(&checker, 0.05);
        let start = vec![-5.0, 0.0];
        let goal = vec![5.0, 0.0];

        let path_1 = rrt_connect(
            &space,
            &checker,
            &mv,
            start.clone(),
            goal.clone(),
            Sampler::unconstrained(&mut rng(10)),
            &params(),
        )
        .expect("gap must be findable");
        let path_2 = rrt_connect(
            &space,
            &checker,
            &mv,
            start,
            goal,
            Sampler::unconstrained(&mut rng(11)),
            &params(),
        )
        .expect("gap must be findable");

        assert_ne!(path_1, path_2);
    }

    /// End-to-end on [`Se3Space`], not just [`RealVectorSpace`]: this is
    /// the case that exercises `interpolate`'s slerp and `distance`'s
    /// weighted translation-plus-rotation metric inside the planner's own
    /// tree-growth and nearest-neighbour code, not just in `se3`'s own
    /// property tests.
    #[test]
    fn runs_end_to_end_on_se3_space() {
        let space = Se3Space::new([(-5.0, 5.0), (-5.0, 5.0), (-5.0, 5.0)], 1.0).unwrap();
        let always_valid = |_: &Se3State| true;
        let mv = DiscreteMotionValidator::new(&always_valid, 0.1);
        let start = Se3State {
            translation: [-3.0, 0.0, 0.0],
            rotation: [1.0, 0.0, 0.0, 0.0],
        };
        let goal = Se3State {
            translation: [3.0, 0.0, 0.0],
            rotation: [0.0, 1.0, 0.0, 0.0],
        };

        let path = rrt_connect(
            &space,
            &always_valid,
            &mv,
            start.clone(),
            goal.clone(),
            Sampler::unconstrained(&mut rng(20)),
            &params(),
        )
        .expect("open SE(3) space must be solvable");

        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
        assert!(path.len() >= 2);
        for pair in path.windows(2) {
            assert!(
                mv.is_motion_valid(&space, &pair[0], &pair[1]),
                "invalid segment {:?} -> {:?}",
                pair[0],
                pair[1]
            );
        }
    }

    /// End-to-end on a [`CompoundSpace`] mixing a `RealVectorSpace`
    /// subspace (prismatic-like joints), a `So2Space` subspace (a
    /// continuous joint) and an `Se3Space` subspace (a floating joint) --
    /// the case the whole exercise was for: a `JointModelGroup` mixing
    /// joint types is exactly this, and nothing in `nn` or `rrt_connect`
    /// needed to change to support it, only `StateSpace` needed to become
    /// object-safe (see `space`'s "Object safety" doc section).
    #[test]
    fn runs_end_to_end_on_compound_space() {
        let space = CompoundSpace::new(vec![
            (
                CompoundSpace::real_vector(
                    RealVectorSpace::new(vec![(-5.0, 5.0), (-5.0, 5.0)]).unwrap(),
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
        .unwrap();
        let always_valid = |_: &Vec<CompoundValue>| true;
        let mv = DiscreteMotionValidator::new(&always_valid, 0.1);
        let start = vec![
            CompoundValue::RealVector(vec![-3.0, -3.0]),
            CompoundValue::So2(3.0),
            CompoundValue::Se3(Se3State {
                translation: [-3.0, 0.0, 0.0],
                rotation: [1.0, 0.0, 0.0, 0.0],
            }),
        ];
        let goal = vec![
            CompoundValue::RealVector(vec![3.0, 3.0]),
            CompoundValue::So2(-3.0),
            CompoundValue::Se3(Se3State {
                translation: [3.0, 0.0, 0.0],
                rotation: [0.0, 1.0, 0.0, 0.0],
            }),
        ];

        let path = rrt_connect(
            &space,
            &always_valid,
            &mv,
            start.clone(),
            goal.clone(),
            Sampler::unconstrained(&mut rng(21)),
            &params(),
        )
        .expect("open compound space must be solvable");

        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
        assert!(path.len() >= 2);
        for pair in path.windows(2) {
            assert!(
                mv.is_motion_valid(&space, &pair[0], &pair[1]),
                "invalid segment {:?} -> {:?}",
                pair[0],
                pair[1]
            );
        }
    }
}
