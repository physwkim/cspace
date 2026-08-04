// Copyright (c) 2016, Southwest Research Institute
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros-industrial/stomp @ b1a87c80f7338caae25a5c689b876da15492aa75:
//   include/stomp/stomp.h
//   src/stomp.cpp

//! `stomp::Stomp`: the optimizer loop itself, driving a [`Task`]
//! implementation through noisy-rollout generation, cost evaluation, and
//! convex-combination parameter updates.
//!
//! # `Stomp::cancel()`'s thread-safety, and why this port splits it out
//!
//! Upstream documents `Stomp::cancel()` as thread-safe: `proceed_` is a
//! `std::atomic<bool>`, callable from another thread while `solve()` runs.
//! `solve()` takes `&mut self` for its whole duration in this port (it
//! mutates nearly every field) -- no other thread can hold even a `&Stomp`
//! to the same value while that borrow is alive, so upstream's own
//! `Stomp::cancel()` (called directly on the same object from another
//! thread) has no safe Rust translation as a plain method. [`CancelHandle`]
//! is the structural fix: [`Stomp::cancel_handle`] clones the underlying
//! `Arc<AtomicBool>` *before* `solve()` is called (an immutable borrow that
//! ends immediately), and the resulting handle -- not `Stomp` itself -- is
//! what a second thread holds and calls `.cancel()` on. [`Stomp::cancel`]
//! itself is kept too, for same-thread API-shape fidelity, but is only
//! meaningfully callable sequentially (e.g. before a subsequent `solve()`
//! call), not concurrently with an in-flight one.
//!
//! # `Task`'s two out-parameter shapes
//!
//! See `task`'s own module doc.
//!
//! # No upstream reference test with value assertions, closing the gap with `test/stomp_3dof.cpp`
//!
//! `test/utest.cpp` is gtest boilerplate; `test/stomp_3dof.cpp`'s own
//! `DummyTask` generates noise via `rand() % RAND_MAX` seeded by `srand(1)`
//! -- libc's PRNG, whose exact output stream is not standardized and not a
//! bit-exact-reproducible ground truth across platforms/toolchains, so it
//! is not a value oracle either. This port's own tests (below) port
//! `stomp_3dof.cpp`'s *structure* -- a `DummyTask` that scores a trajectory
//! against a bias with a threshold, `compareDiff`'s threshold check, the
//! 3-DOF linear-interpolation scenario -- but generate noise via
//! `moveit_sampling::MultivariateGaussian::sample_with_covariance` (per
//! this round's brief: STOMP is the covariance-using caller of that class)
//! seeded by a fixed `rand_chacha::ChaCha8Rng`, so the test is
//! deterministic without claiming to reproduce upstream's specific libc
//! PRNG stream. The assertion this port keeps from upstream is the
//! meaningful one: `solve()` converges to within `BIAS_THRESHOLD` of the
//! bias trajectory it was scored against.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use nalgebra::DMatrix;

use crate::task::Task;
use crate::utils::{
    DerivativeOrder, FINITE_DIFF_RULE_LENGTH, Rollout, StompConfiguration,
    TrajectoryInitialization, full_piv_lu_try_inverse_or_empty, generate_finite_difference_matrix,
};

/// Minimum cost difference allowed during probability calculation.
const MIN_COST_DIFFERENCE: f64 = 1e-8;
/// Minimum control cost weight allowed before it is treated as "off".
const MIN_CONTROL_COST_WEIGHT: f64 = 1e-8;

/// A thread-safe handle to cancel an in-flight [`Stomp::solve`]. See this
/// module's own doc, "`Stomp::cancel()`'s thread-safety".
#[derive(Clone)]
pub struct CancelHandle(Arc<AtomicBool>);

impl CancelHandle {
    /// Requests cancellation. `solve()` checks this at the start of every
    /// iteration and, within `generate_noisy_rollouts`, before generating
    /// each noisy rollout -- the same points upstream's `proceed_` check
    /// gates.
    pub fn cancel(&self) {
        self.0.store(false, Ordering::SeqCst);
    }
}

/// `computeLinearInterpolation`. Returns the trajectory rather than writing
/// through an out-parameter -- see `utils`' module doc, "Deviation: return
/// values, not out-parameters" (the same convention, applied here too).
///
/// Upstream computes `dtheta` via `(last[i] - first[i]) / (num_timesteps -
/// 1)` on `int num_timesteps`; this port divides using `num_timesteps as
/// f64 - 1.0` (float subtraction) rather than `(num_timesteps - 1) as
/// f64` (integer subtraction first) so that `num_timesteps == 0` produces
/// the same `-1.0` divisor C++'s signed `int` gives, instead of a `usize`
/// underflow panic -- the inner loop never runs for `num_timesteps == 0`
/// either way (`0..0` is empty), so this only changes an unused
/// divide-by-`-1.0` from "well-defined but discarded" to "well-defined but
/// discarded", matching upstream's own dead computation exactly rather than
/// panicking on it.
fn compute_linear_interpolation(first: &[f64], last: &[f64], num_timesteps: usize) -> DMatrix<f64> {
    let mut trajectory_joints = DMatrix::zeros(first.len(), num_timesteps);
    for i in 0..first.len() {
        let dtheta = (last[i] - first[i]) / (num_timesteps as f64 - 1.0);
        for j in 0..num_timesteps {
            trajectory_joints[(i, j)] = first[i] + j as f64 * dtheta;
        }
    }
    trajectory_joints
}

/// `computeCubicInterpolation`. Returns the trajectory rather than writing
/// through an out-parameter. Upstream's version does *not* resize its
/// `trajectory_joints` out-parameter itself (unlike
/// [`compute_linear_interpolation`]'s upstream, which does) -- it relies on
/// the caller (`computeInitialTrajectory`) having already sized
/// `parameters_optimized_` to `(num_dimensions, num_timesteps)` beforehand.
/// Converting to a return value sidesteps that asymmetry entirely: this
/// port always allocates fresh at `(first.len(), num_points)`, the same
/// dimensions either function produces.
fn compute_cubic_interpolation(
    first: &[f64],
    last: &[f64],
    num_points: usize,
    dt: f64,
) -> DMatrix<f64> {
    let mut trajectory_joints = DMatrix::zeros(first.len(), num_points);
    let total_time = (num_points as f64 - 1.0) * dt;
    for i in 0..first.len() {
        let c0 = first[i];
        let c2 = (3.0 / total_time.powi(2)) * (last[i] - first[i]);
        let c3 = (-2.0 / total_time.powi(3)) * (last[i] - first[i]);
        for j in 0..num_points {
            let t = j as f64 * dt;
            trajectory_joints[(i, j)] = c0 + c2 * t.powi(2) + c3 * t.powi(3);
        }
    }
    trajectory_joints
}

/// `computeMinCostTrajectory`. `None` where upstream returns `false` (a
/// non-square `control_cost_matrix_R_padded` -- structurally unreachable
/// through [`Stomp`]'s own construction, which always builds it square, but
/// kept as `Option` rather than an unchecked assumption since the check
/// costs nothing and upstream itself performs it).
fn compute_min_cost_trajectory(
    first: &[f64],
    last: &[f64],
    control_cost_matrix_r_padded: &DMatrix<f64>,
    inv_control_cost_matrix_r: &DMatrix<f64>,
) -> Option<DMatrix<f64>> {
    if control_cost_matrix_r_padded.nrows() != control_cost_matrix_r_padded.ncols() {
        return None;
    }

    let timesteps = control_cost_matrix_r_padded.nrows() - 2 * (FINITE_DIFF_RULE_LENGTH - 1);
    let start_index_padded = FINITE_DIFF_RULE_LENGTH - 1;
    let end_index_padded = start_index_padded + timesteps - 1;
    let ones = nalgebra::RowDVector::<f64>::from_element(FINITE_DIFF_RULE_LENGTH - 1, 1.0);

    let mut trajectory_joints = DMatrix::zeros(first.len(), timesteps);
    for d in 0..first.len() {
        let block_first = control_cost_matrix_r_padded.view(
            (0, start_index_padded),
            (FINITE_DIFF_RULE_LENGTH - 1, timesteps),
        );
        let mut linear_control_cost = (first[d] * &ones * block_first).transpose();

        let block_last = control_cost_matrix_r_padded.view(
            (end_index_padded + 1, start_index_padded),
            (FINITE_DIFF_RULE_LENGTH - 1, timesteps),
        );
        linear_control_cost += (last[d] * &ones * block_last).transpose();
        linear_control_cost *= 2.0;

        let result_col = -0.5 * inv_control_cost_matrix_r * &linear_control_cost;
        for t in 0..timesteps {
            trajectory_joints[(d, t)] = result_col[t];
        }
        trajectory_joints[(d, 0)] = first[d];
        trajectory_joints[(d, timesteps - 1)] = last[d];
    }

    Some(trajectory_joints)
}

/// `computeParametersControlCosts`. Returns the control-cost matrix rather
/// than writing through an out-parameter.
fn compute_parameters_control_costs(
    parameters: &DMatrix<f64>,
    dt: f64,
    control_cost_weight: f64,
    control_cost_matrix_r: &DMatrix<f64>,
) -> DMatrix<f64> {
    let num_timesteps = parameters.ncols();
    let mut control_costs = DMatrix::zeros(parameters.nrows(), num_timesteps);
    for d in 0..parameters.nrows() {
        let row = parameters.row(d);
        let cost = (row * (control_cost_matrix_r * row.transpose()))[(0, 0)];
        control_costs.row_mut(d).fill(0.5 * (1.0 / dt) * cost);
    }

    let max_coeff = control_costs.max();
    control_costs /= if max_coeff > 1e-8 { max_coeff } else { 1.0 };
    control_costs *= control_cost_weight;
    control_costs
}

/// `stomp::Stomp`.
///
/// # `'a`: round 23, not upstream
///
/// `stomp::TaskPtr` is a `std::shared_ptr<Task>` with no lifetime to track.
/// A caller building a real [`Task`] from borrowed data (a `RobotModel`, a
/// `JointModelGroup` -- see `moveit_planners_stomp::filter_functions::
/// enforce_position_bounds`) cannot produce a `'static` one, so `Stomp`
/// carries the same borrow the way [`crate::task::Task`]'s own
/// implementations already must: `Box<dyn Task + 'a>`, not an implicit
/// `Box<dyn Task + 'static>`.
pub struct Stomp<'a> {
    task: Box<dyn Task + 'a>,
    config: StompConfiguration,
    proceed: Arc<AtomicBool>,
    current_iteration: i32,

    parameters_valid: bool,
    parameters_valid_prev: bool,
    parameters_total_cost: f64,
    /// Uninitialized in upstream until `solve()`'s own `= max()` assignment
    /// runs (never read before then); this port seeds it at construction
    /// with the same value `solve()` assigns, so there is no window where
    /// it holds anything else.
    current_lowest_cost: f64,
    parameters_optimized: DMatrix<f64>,

    parameters_updates: DMatrix<f64>,
    parameters_state_costs: nalgebra::DVector<f64>,
    parameters_control_costs: DMatrix<f64>,

    noisy_rollouts: Vec<Rollout>,
    reused_rollouts: Vec<Rollout>,
    num_active_rollouts: usize,

    start_index_padded: usize,
    finite_diff_matrix_a_padded: DMatrix<f64>,
    control_cost_matrix_r_padded: DMatrix<f64>,
    control_cost_matrix_r: DMatrix<f64>,
    inv_control_cost_matrix_r: DMatrix<f64>,
}

impl<'a> Stomp<'a> {
    /// `Stomp(config, task)`.
    pub fn new(config: StompConfiguration, task: Box<dyn Task + 'a>) -> Self {
        let mut stomp = Self {
            task,
            config,
            proceed: Arc::new(AtomicBool::new(true)),
            current_iteration: 0,
            parameters_valid: false,
            parameters_valid_prev: false,
            parameters_total_cost: 0.0,
            current_lowest_cost: f64::MAX,
            parameters_optimized: DMatrix::zeros(0, 0),
            parameters_updates: DMatrix::zeros(0, 0),
            parameters_state_costs: nalgebra::DVector::zeros(0),
            parameters_control_costs: DMatrix::zeros(0, 0),
            noisy_rollouts: Vec::new(),
            reused_rollouts: Vec::new(),
            num_active_rollouts: 0,
            start_index_padded: 0,
            finite_diff_matrix_a_padded: DMatrix::zeros(0, 0),
            control_cost_matrix_r_padded: DMatrix::zeros(0, 0),
            control_cost_matrix_r: DMatrix::zeros(0, 0),
            inv_control_cost_matrix_r: DMatrix::zeros(0, 0),
        };
        stomp.reset_variables();
        stomp
    }

    /// `setConfig`: replaces the configuration and re-derives every
    /// internal matrix from it.
    pub fn set_config(&mut self, config: StompConfiguration) {
        self.config = config;
        self.reset_variables();
    }

    /// `clear`: resets all internal variables without changing the
    /// configuration.
    pub fn clear(&mut self) {
        self.reset_variables();
    }

    /// `cancel`. See this module's own doc, "`Stomp::cancel()`'s
    /// thread-safety", for why this is only meaningfully callable
    /// same-thread/sequentially -- use [`Stomp::cancel_handle`] to cancel a
    /// `solve()` call in flight on another thread.
    pub fn cancel(&self) -> bool {
        self.proceed.store(false, Ordering::SeqCst);
        true
    }

    /// A cloneable, thread-safe handle that can cancel a `solve()` call
    /// after it has started on another thread. Obtain this *before* calling
    /// `solve()` -- `solve()` borrows `&mut self` for its whole duration.
    pub fn cancel_handle(&self) -> CancelHandle {
        CancelHandle(Arc::clone(&self.proceed))
    }

    /// `solve(first, last, parameters_optimized)`: computes an initial
    /// trajectory via `Stomp::compute_initial_trajectory`, then optimizes
    /// it. Upstream's `Eigen::VectorXd` overload of the same name is not
    /// ported separately: converting a `DVector<f64>` to `&[f64]` is
    /// `.as_slice()` at the call site, so it carries no separate behavior
    /// worth its own method in Rust the way it does in C++ (where
    /// `Eigen::VectorXd` and `std::vector<double>` are unrelated types).
    ///
    /// Returns `(valid, parameters_optimized)`, upstream's `bool` return
    /// and `Eigen::MatrixXd&` out-parameter respectively.
    ///
    /// # Upstream quirk, preserved: `compute_initial_trajectory`'s failure
    /// is logged, not acted on
    ///
    /// If `compute_initial_trajectory` returns `false` (only possible for
    /// [`TrajectoryInitialization::MinimumControlCost`] with a non-square
    /// padded control-cost matrix -- structurally unreachable, see
    /// `compute_min_cost_trajectory`'s own doc), upstream logs an error and
    /// calls `solve()` anyway against whatever `parameters_optimized_`
    /// currently holds. This port does the same: the `bool` result is
    /// discarded here, matching upstream's own dead error-handling path.
    pub fn solve_from_endpoints(&mut self, first: &[f64], last: &[f64]) -> (bool, DMatrix<f64>) {
        let _ = self.compute_initial_trajectory(first, last);
        let initial = self.parameters_optimized.clone();
        self.solve(&initial)
    }

    /// `solve(initial_parameters, parameters_optimized)`.
    ///
    /// # Upstream quirk, preserved: `initial_parameters` only seeds the
    /// optimizer when `parameters_optimized` is exactly zero
    ///
    /// Upstream: `if (parameters_optimized_.isZero()) { parameters_optimized_
    /// = initial_parameters; }`. On a freshly constructed (or just-`clear`ed)
    /// [`Stomp`], `parameters_optimized` starts at all-zero, so
    /// `initial_parameters` does seed it. Calling `solve` a *second* time on
    /// the same `Stomp` without resetting leaves `parameters_optimized`
    /// non-zero from the first call, so `initial_parameters` is silently
    /// **not** used to seed the second run -- the previous run's result is
    /// reused as the starting point instead, though `initial_parameters`'s
    /// shape is still validated below either way. Preserved exactly, not
    /// "fixed": this is a real, if surprising, documented-by-observation
    /// upstream behavior, not a translation bug.
    ///
    /// Also preserved: upstream's dimension check is written as an outer
    /// `if`/`else` whose `else` branch re-checks a condition the outer `if`'s
    /// negation already rules out (`cols() != num_timesteps` cannot be true
    /// inside the `else` of `rows() != num_dimensions || cols() !=
    /// num_timesteps`) -- provably unreachable, so this port collapses it to
    /// the one reachable check rather than translating dead code.
    pub fn solve(&mut self, initial_parameters: &DMatrix<f64>) -> (bool, DMatrix<f64>) {
        if self.parameters_optimized.iter().all(|&v| v == 0.0) {
            self.parameters_optimized = initial_parameters.clone();
        }

        if initial_parameters.nrows() != self.config.num_dimensions
            || initial_parameters.ncols() != self.config.num_timesteps
        {
            return (false, self.parameters_optimized.clone());
        }

        self.current_iteration = 1;
        let mut valid_iterations: usize = 0;
        self.current_lowest_cost = f64::MAX;

        if !self.compute_optimized_cost() {
            return (false, self.parameters_optimized.clone());
        }

        self.parameters_valid_prev = self.parameters_valid;
        while (self.current_iteration as usize) <= self.config.num_iterations
            && self.run_single_iteration()
        {
            if self.parameters_valid {
                valid_iterations += 1;
            } else {
                valid_iterations = 0;
            }

            if valid_iterations > self.config.num_iterations_after_valid {
                break;
            }

            self.current_iteration += 1;
        }

        let parameters_optimized = self.parameters_optimized.clone();
        self.task.done(
            self.parameters_valid,
            self.current_iteration,
            self.current_lowest_cost,
            &parameters_optimized,
        );

        (self.parameters_valid, parameters_optimized)
    }

    /// `resetVariables`.
    fn reset_variables(&mut self) {
        self.proceed.store(true, Ordering::SeqCst);
        self.parameters_total_cost = 0.0;
        self.parameters_valid = false;
        self.num_active_rollouts = 0;
        self.current_iteration = 0;

        if self.config.max_rollouts <= self.config.num_rollouts {
            self.config.max_rollouts = self.config.num_rollouts + 1;
        }

        let d = self.config.num_dimensions;
        let t = self.config.num_timesteps;

        let rollout = Rollout::new(d, t);
        self.noisy_rollouts = vec![rollout.clone(); self.config.max_rollouts];
        self.reused_rollouts = vec![rollout; self.config.max_rollouts];

        self.parameters_updates = DMatrix::zeros(d, t);
        self.parameters_control_costs = DMatrix::zeros(d, t);
        self.parameters_state_costs = nalgebra::DVector::zeros(t);
        self.parameters_optimized = DMatrix::zeros(d, t);

        self.start_index_padded = FINITE_DIFF_RULE_LENGTH - 1;
        let num_timesteps_padded = t + 2 * (FINITE_DIFF_RULE_LENGTH - 1);
        self.finite_diff_matrix_a_padded = generate_finite_difference_matrix(
            num_timesteps_padded,
            DerivativeOrder::Acceleration,
            self.config.delta_t,
        );

        self.control_cost_matrix_r_padded = self.config.delta_t
            * self.finite_diff_matrix_a_padded.transpose()
            * &self.finite_diff_matrix_a_padded;
        self.control_cost_matrix_r = self
            .control_cost_matrix_r_padded
            .view((self.start_index_padded, self.start_index_padded), (t, t))
            .into_owned();
        self.inv_control_cost_matrix_r =
            full_piv_lu_try_inverse_or_empty(self.control_cost_matrix_r.clone()).expect(
                "control_cost_matrix_R is invertible by STOMP's own algorithmic premise -- see \
                 generate_smoothing_matrix's own doc for the same reasoning",
            );

        // "Applying scale factor to ensure that max(R^-1)==1": upstream
        // takes std::abs(maxCoeff()) -- the absolute value of the single
        // largest raw entry (which can be negative for an SPD matrix's
        // off-diagonal entries), not the max of all entries' absolute
        // values (`cwiseAbs().maxCoeff()`, a different computation).
        let max_val = self.inv_control_cost_matrix_r.max().abs();
        self.control_cost_matrix_r_padded *= max_val;
        self.control_cost_matrix_r *= max_val;
        self.inv_control_cost_matrix_r /= max_val;
    }

    /// `computeInitialTrajectory`.
    fn compute_initial_trajectory(&mut self, first: &[f64], last: &[f64]) -> bool {
        match self.config.initialization_method {
            TrajectoryInitialization::CubicPolynomialInterpolation => {
                self.parameters_optimized = compute_cubic_interpolation(
                    first,
                    last,
                    self.config.num_timesteps,
                    self.config.delta_t,
                );
                true
            }
            TrajectoryInitialization::LinearInterpolation => {
                self.parameters_optimized =
                    compute_linear_interpolation(first, last, self.config.num_timesteps);
                true
            }
            TrajectoryInitialization::MinimumControlCost => {
                match compute_min_cost_trajectory(
                    first,
                    last,
                    &self.control_cost_matrix_r_padded,
                    &self.inv_control_cost_matrix_r,
                ) {
                    Some(trajectory) => {
                        self.parameters_optimized = trajectory;
                        true
                    }
                    None => false,
                }
            }
        }
    }

    /// `runSingleIteration`. Preserves upstream's exact short-circuit call
    /// order (`generate_noisy_rollouts`, `compute_noisy_rollouts_costs`,
    /// `filter_noisy_rollouts`, `compute_probabilities`,
    /// `update_parameters`, `compute_optimized_cost`) -- notably *not* the
    /// order `stomp.h`'s own method declarations are listed in, which lists
    /// `filterNoisyRollouts` before `computeNoisyRolloutsCosts`; the .cpp's
    /// actual call chain is authoritative.
    fn run_single_iteration(&mut self) -> bool {
        if !self.proceed.load(Ordering::SeqCst) {
            return false;
        }

        let proceed = self.generate_noisy_rollouts()
            && self.compute_noisy_rollouts_costs()
            && self.filter_noisy_rollouts()
            && self.compute_probabilities()
            && self.update_parameters()
            && self.compute_optimized_cost();

        self.task.post_iteration(
            0,
            self.config.num_timesteps,
            self.current_iteration,
            self.current_lowest_cost,
            &self.parameters_optimized,
        );

        proceed
    }

    /// `generateNoisyRollouts`.
    ///
    /// # Upstream quirk, preserved: rollout 0's cost never widens the reuse
    /// min/max scan
    ///
    /// The min/max scan over stored rollouts' `total_cost` starts at `r =
    /// 1`, not `r = 0` -- index 0's cost is never inspected when
    /// establishing `min_cost`/`max_cost`, even though index 0 *is* later
    /// included (from `r = 0`) when computing each rollout's weighted
    /// probability against those bounds. Preserved exactly.
    ///
    /// # Upstream quirk, preserved: `max_cost`'s seed is `min()`, not
    /// `lowest()`
    ///
    /// `std::numeric_limits<double>::min()` is the smallest *positive*
    /// normal `double` (~2.2e-308), not negative infinity -- a well-known
    /// C++ trap (`lowest()` is the most-negative value, a different call).
    /// `f64::MIN_POSITIVE` is the faithful Rust equivalent; `f64::MIN`
    /// would silently "fix" this quirk instead of reproducing it.
    fn generate_noisy_rollouts(&mut self) -> bool {
        let h = self.config.exponentiated_cost_sensitivity;
        let rollouts_stored = self.num_active_rollouts.saturating_sub(1);
        let rollouts_generate = self.config.num_rollouts;
        let rollouts_total = rollouts_generate + rollouts_stored + 1;
        let rollouts_reuse = if rollouts_total < self.config.max_rollouts {
            rollouts_stored
        } else {
            self.config.max_rollouts - (rollouts_generate + 1)
        };

        if rollouts_reuse > 0 {
            let mut min_cost = f64::MAX;
            let mut max_cost = f64::MIN_POSITIVE;
            for r in 1..rollouts_stored {
                let c = self.noisy_rollouts[r].total_cost;
                if c < min_cost {
                    min_cost = c;
                }
                if c > max_cost {
                    max_cost = c;
                }
            }

            let mut cost_denom = max_cost - min_cost;
            if cost_denom < 1e-8 {
                cost_denom = 1e-8;
            }

            let mut rollout_cost_sorter: Vec<(f64, usize)> = Vec::with_capacity(rollouts_stored);
            for r in 0..rollouts_stored {
                self.noisy_rollouts[r].noise =
                    &self.noisy_rollouts[r].parameters_noise - &self.parameters_optimized;
                let cost_prob =
                    (-h * (self.noisy_rollouts[r].total_cost - min_cost) / cost_denom).exp();
                let weighted_prob = cost_prob * self.noisy_rollouts[r].importance_weight;
                rollout_cost_sorter.push((-weighted_prob, r));
            }

            rollout_cost_sorter.sort_by(|a, b| {
                a.0.partial_cmp(&b.0)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then(a.1.cmp(&b.1))
            });

            for (r, &(_, reuse_index)) in rollout_cost_sorter.iter().enumerate() {
                self.reused_rollouts[r] = self.noisy_rollouts[reuse_index].clone();
            }

            for r in 0..rollouts_reuse {
                self.noisy_rollouts[rollouts_generate + r] = self.reused_rollouts[r].clone();
            }
        }

        self.noisy_rollouts[rollouts_generate + rollouts_reuse].parameters_noise =
            self.parameters_optimized.clone();
        self.noisy_rollouts[rollouts_generate + rollouts_reuse]
            .noise
            .fill(0.0);
        self.noisy_rollouts[rollouts_generate + rollouts_reuse].state_costs =
            self.parameters_state_costs.clone();
        self.noisy_rollouts[rollouts_generate + rollouts_reuse].control_costs =
            self.parameters_control_costs.clone();

        for r in 0..rollouts_generate {
            if !self.proceed.load(Ordering::SeqCst) {
                return false;
            }
            match self.task.generate_noisy_parameters(
                &self.parameters_optimized,
                0,
                self.config.num_timesteps,
                self.current_iteration,
                r as i32,
            ) {
                Some((parameters_noise, noise)) => {
                    self.noisy_rollouts[r].parameters_noise = parameters_noise;
                    self.noisy_rollouts[r].noise = noise;
                }
                None => return false,
            }
        }

        self.num_active_rollouts = rollouts_reuse + rollouts_generate + 1;
        true
    }

    /// `filterNoisyRollouts`.
    fn filter_noisy_rollouts(&mut self) -> bool {
        for r in 0..self.config.num_rollouts {
            if !self.proceed.load(Ordering::SeqCst) {
                return false;
            }

            let (success, filtered) = self.task.filter_noisy_parameters(
                0,
                self.config.num_timesteps,
                self.current_iteration,
                r as i32,
                &mut self.noisy_rollouts[r].parameters_noise,
            );
            if !success {
                return false;
            }

            if filtered {
                self.noisy_rollouts[r].noise =
                    &self.noisy_rollouts[r].parameters_noise - &self.parameters_optimized;
            }
        }
        true
    }

    /// `computeNoisyRolloutsCosts`.
    fn compute_noisy_rollouts_costs(&mut self) -> bool {
        let valid = self.compute_rollouts_state_costs() && self.compute_rollouts_control_costs();
        if valid {
            let num_dimensions = self.config.num_dimensions;
            for r in 0..self.num_active_rollouts {
                let rollout = &mut self.noisy_rollouts[r];
                let total_state_cost = rollout.state_costs.sum();
                let mut total_control_cost = 0.0;
                for d in 0..num_dimensions {
                    let ccost = rollout.control_costs.row(d).sum();
                    total_control_cost += ccost;
                    rollout.full_costs[d] = ccost + total_state_cost;
                }
                rollout.total_cost = total_state_cost + total_control_cost;
                for d in 0..num_dimensions {
                    let updated_row =
                        rollout.state_costs.transpose() + rollout.control_costs.row(d);
                    rollout.total_costs.row_mut(d).copy_from(&updated_row);
                }
            }
        }
        valid
    }

    /// `computeRolloutsStateCosts`.
    ///
    /// Upstream computes an `all_valid` out-param on every
    /// `task_->computeNoisyCosts` call but never reads it after the loop --
    /// dead state, kept dead here too (this port's `Task::compute_noisy_costs`
    /// still returns a validity flag per the trait's own out-param
    /// conversion, but the caller here discards it via `_`, matching
    /// upstream's own unused value rather than inventing a use for it).
    fn compute_rollouts_state_costs(&mut self) -> bool {
        let mut proceed = true;
        for r in 0..self.config.num_rollouts {
            if !self.proceed.load(Ordering::SeqCst) {
                proceed = false;
                break;
            }

            match self.task.compute_noisy_costs(
                &self.noisy_rollouts[r].parameters_noise,
                0,
                self.config.num_timesteps,
                self.current_iteration,
                r as i32,
            ) {
                Some((costs, _validity)) => {
                    self.noisy_rollouts[r].state_costs = costs;
                }
                None => {
                    proceed = false;
                    break;
                }
            }
        }
        proceed
    }

    /// `computeRolloutsControlCosts`. Upstream declares an
    /// `Eigen::ArrayXXd Ax;` local that is never read anywhere in the
    /// function body -- genuinely dead, omitted here rather than given a
    /// pointless Rust binding.
    fn compute_rollouts_control_costs(&mut self) -> bool {
        for r in 0..self.num_active_rollouts {
            if self.config.control_cost_weight < MIN_CONTROL_COST_WEIGHT {
                self.noisy_rollouts[r].control_costs.fill(0.0);
            } else {
                self.noisy_rollouts[r].control_costs = compute_parameters_control_costs(
                    &self.noisy_rollouts[r].parameters_noise,
                    self.config.delta_t,
                    self.config.control_cost_weight,
                    &self.control_cost_matrix_r,
                );
            }
        }
        true
    }

    /// `computeProbabilities`.
    fn compute_probabilities(&mut self) -> bool {
        let h = self.config.exponentiated_cost_sensitivity;
        let num_active_rollouts = self.num_active_rollouts;

        for d in 0..self.config.num_dimensions {
            for t in 0..self.config.num_timesteps {
                let mut min_cost = self.noisy_rollouts[0].total_costs[(d, t)];
                let mut max_cost = min_cost;
                for r in 0..num_active_rollouts {
                    let cost = self.noisy_rollouts[r].total_costs[(d, t)];
                    if cost < min_cost {
                        min_cost = cost;
                    }
                    if cost > max_cost {
                        max_cost = cost;
                    }
                }

                let mut denom = max_cost - min_cost;
                if denom < MIN_COST_DIFFERENCE {
                    denom = MIN_COST_DIFFERENCE;
                }

                let mut probl_sum = 0.0;
                for r in 0..num_active_rollouts {
                    let exponent =
                        -h * (self.noisy_rollouts[r].total_costs[(d, t)] - min_cost) / denom;
                    let p = self.noisy_rollouts[r].importance_weight * exponent.exp();
                    self.noisy_rollouts[r].probabilities[(d, t)] = p;
                    probl_sum += p;
                }

                for r in 0..num_active_rollouts {
                    self.noisy_rollouts[r].probabilities[(d, t)] /= probl_sum;
                }
            }

            let mut min_cost = self.noisy_rollouts[0].full_costs[d];
            let mut max_cost = min_cost;
            for r in 1..num_active_rollouts {
                let c = self.noisy_rollouts[r].full_costs[d];
                if c < min_cost {
                    min_cost = c;
                }
                if c > max_cost {
                    max_cost = c;
                }
            }

            let mut denom = max_cost - min_cost;
            if denom < MIN_COST_DIFFERENCE {
                denom = MIN_COST_DIFFERENCE;
            }

            let mut probl_sum = 0.0;
            for r in 0..num_active_rollouts {
                let p = self.noisy_rollouts[r].importance_weight
                    * (-h * (self.noisy_rollouts[r].full_costs[d] - min_cost) / denom).exp();
                self.noisy_rollouts[r].full_probabilities[d] = p;
                probl_sum += p;
            }
            for r in 0..num_active_rollouts {
                self.noisy_rollouts[r].full_probabilities[d] /= probl_sum;
            }
        }

        true
    }

    /// `updateParameters`.
    fn update_parameters(&mut self) -> bool {
        self.parameters_updates.fill(0.0);
        for d in 0..self.config.num_dimensions {
            let mut row_sum = self.parameters_updates.row(d).clone_owned();
            for r in 0..self.num_active_rollouts {
                let contribution = self.noisy_rollouts[r]
                    .noise
                    .row(d)
                    .component_mul(&self.noisy_rollouts[r].probabilities.row(d));
                row_sum += contribution;
            }
            self.parameters_updates.row_mut(d).copy_from(&row_sum);
        }

        let success = self.task.filter_parameter_updates(
            0,
            self.config.num_timesteps,
            self.current_iteration,
            &self.parameters_optimized,
            &mut self.parameters_updates,
        );
        if !success {
            return false;
        }

        self.parameters_optimized += &self.parameters_updates;
        true
    }

    /// `computeOptimizedCost`.
    fn compute_optimized_cost(&mut self) -> bool {
        self.parameters_total_cost = 0.0;
        if self.config.control_cost_weight > MIN_CONTROL_COST_WEIGHT {
            self.parameters_control_costs = compute_parameters_control_costs(
                &self.parameters_optimized,
                self.config.delta_t,
                self.config.control_cost_weight,
                &self.control_cost_matrix_r,
            );
            // "rowwise().sum().sum()": sum each row, then sum those sums --
            // the same total as summing every entry once. This port takes
            // that total directly; bit-exact reproduction of Eigen's own
            // internal reduction order is not attempted (or attemptable)
            // here regardless of which nalgebra method is used.
            self.parameters_total_cost = self.parameters_control_costs.sum();
        }

        match self.task.compute_costs(
            &self.parameters_optimized,
            0,
            self.config.num_timesteps,
            self.current_iteration,
        ) {
            Some((state_costs, validity)) => {
                self.parameters_total_cost += state_costs.sum();
                self.parameters_state_costs = state_costs;
                self.parameters_valid = validity;
            }
            None => return false,
        }

        if self.current_lowest_cost > self.parameters_total_cost {
            self.current_lowest_cost = self.parameters_total_cost;
            self.parameters_valid_prev = self.parameters_valid;
        } else if self.parameters_valid_prev {
            self.parameters_optimized -= &self.parameters_updates;
            self.parameters_valid = self.parameters_valid_prev;
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::generate_smoothing_matrix;
    use moveit_sampling::MultivariateGaussian;
    use nalgebra::DVector;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    const NUM_DIMENSIONS: usize = 3;
    const NUM_TIMESTEPS: usize = 20;
    const DELTA_T: f64 = 0.1;
    const START_POS: [f64; 3] = [1.4, 1.4, 0.5];
    const END_POS: [f64; 3] = [-1.25, 1.0, -0.26];
    const BIAS_THRESHOLD: [f64; 3] = [0.050, 0.050, 0.050];
    const STD_DEV: [f64; 3] = [1.0, 1.0, 1.0];

    /// `DummyTask`. Upstream's noise is `rand() % RAND_MAX` seeded by
    /// `srand(1)` -- libc's PRNG stream is not a portable, bit-exact ground
    /// truth (see this module's own doc, "No upstream reference test with
    /// value assertions"). This port generates noise via
    /// `MultivariateGaussian::sample_with_covariance` (one generator per
    /// dimension, diagonal covariance `std_dev[d]^2 * I` so each timestep is
    /// iid `N(0, std_dev[d]^2)`, matching `noise(d, t) = rand_noise *
    /// std_dev_[d]`'s per-dimension scaling) seeded by a fixed
    /// `ChaCha8Rng`, per this round's brief.
    struct DummyTask {
        parameters_bias: DMatrix<f64>,
        bias_thresholds: Vec<f64>,
        noise_generators: Vec<MultivariateGaussian>,
        rng: ChaCha8Rng,
        smoothing_m: DMatrix<f64>,
    }

    impl DummyTask {
        fn new(
            parameters_bias: DMatrix<f64>,
            bias_thresholds: &[f64],
            std_dev: &[f64],
            seed: u64,
        ) -> Self {
            let num_timesteps = parameters_bias.ncols();
            let smoothing_m = generate_smoothing_matrix(num_timesteps, 1.0)
                .expect("smoothing matrix invertible for this test's config");
            let noise_generators = std_dev
                .iter()
                .map(|&sigma| {
                    let mean = DVector::zeros(num_timesteps);
                    let covariance =
                        DMatrix::identity(num_timesteps, num_timesteps) * (sigma * sigma);
                    MultivariateGaussian::new(mean, covariance)
                        .expect("diagonal covariance with sigma > 0 is positive-definite")
                })
                .collect();
            Self {
                parameters_bias,
                bias_thresholds: bias_thresholds.to_vec(),
                noise_generators,
                rng: ChaCha8Rng::seed_from_u64(seed),
                smoothing_m,
            }
        }
    }

    impl Task for DummyTask {
        fn generate_noisy_parameters(
            &mut self,
            parameters: &DMatrix<f64>,
            _start_timestep: usize,
            num_timesteps: usize,
            _iteration_number: i32,
            _rollout_number: i32,
        ) -> Option<(DMatrix<f64>, DMatrix<f64>)> {
            let mut noise = DMatrix::zeros(parameters.nrows(), num_timesteps);
            let mut row = DVector::zeros(num_timesteps);
            for d in 0..parameters.nrows() {
                self.noise_generators[d].sample_with_covariance(&mut row, &mut self.rng);
                noise.row_mut(d).copy_from(&row.transpose());
            }
            let parameters_noise = parameters + &noise;
            Some((parameters_noise, noise))
        }

        fn compute_noisy_costs(
            &mut self,
            parameters: &DMatrix<f64>,
            _start_timestep: usize,
            num_timesteps: usize,
            _iteration_number: i32,
            _rollout_number: i32,
        ) -> Option<(DVector<f64>, bool)> {
            let mut costs = DVector::zeros(num_timesteps);
            let mut validity = true;
            for t in 0..num_timesteps {
                let mut cost = 0.0;
                for d in 0..parameters.nrows() {
                    let diff = (parameters[(d, t)] - self.parameters_bias[(d, t)]).abs();
                    if diff > self.bias_thresholds[d].abs() {
                        cost += diff;
                        validity = false;
                    }
                }
                costs[t] = cost;
            }
            Some((costs, validity))
        }

        fn compute_costs(
            &mut self,
            parameters: &DMatrix<f64>,
            start_timestep: usize,
            num_timesteps: usize,
            iteration_number: i32,
        ) -> Option<(DVector<f64>, bool)> {
            self.compute_noisy_costs(
                parameters,
                start_timestep,
                num_timesteps,
                iteration_number,
                -1,
            )
        }

        fn filter_parameter_updates(
            &mut self,
            _start_timestep: usize,
            _num_timesteps: usize,
            _iteration_number: i32,
            _parameters: &DMatrix<f64>,
            updates: &mut DMatrix<f64>,
        ) -> bool {
            for d in 0..updates.nrows() {
                let smoothed = (&self.smoothing_m * updates.row(d).transpose()).transpose();
                updates.row_mut(d).copy_from(&smoothed);
            }
            true
        }
    }

    /// `interpolate` (`test/stomp_3dof.cpp`'s own local helper, a second,
    /// separately-maintained copy of the same linear-interpolation
    /// algorithm as [`compute_linear_interpolation`]). Kept as its own
    /// duplicate here too, rather than calling
    /// [`compute_linear_interpolation`] directly: that function is itself
    /// one of the things under test (via
    /// [`TrajectoryInitialization::LinearInterpolation`]), and using it to
    /// build the bias trajectory these tests score convergence against
    /// would let a latent bug in it silently cancel out of the
    /// `LinearInterpolation`-initialized test cases.
    fn interpolate(start: &[f64], end: &[f64], num_timesteps: usize) -> DMatrix<f64> {
        let dimensions = start.len();
        let mut traj = DMatrix::zeros(dimensions, num_timesteps);
        for d in 0..dimensions {
            let delta = (end[d] - start[d]) / (num_timesteps as f64 - 1.0);
            for t in 0..num_timesteps {
                traj[(d, t)] = start[d] + t as f64 * delta;
            }
        }
        traj
    }

    /// `compareDiff`.
    fn compare_diff(optimized: &DMatrix<f64>, desired: &DMatrix<f64>, thresholds: &[f64]) -> bool {
        for d in 0..optimized.nrows() {
            for t in 0..optimized.ncols() {
                if (optimized[(d, t)] - desired[(d, t)]).abs() > thresholds[d] {
                    return false;
                }
            }
        }
        true
    }

    /// `create3DOFConfiguration`. See [`StompConfiguration`]'s own doc for
    /// why `exponentiated_cost_sensitivity` is `0.5` here rather than a
    /// value read out of upstream's own test (which leaves it
    /// uninitialized).
    fn create_3dof_configuration(num_timesteps: usize) -> StompConfiguration {
        StompConfiguration {
            num_iterations: 40,
            num_iterations_after_valid: 0,
            num_timesteps,
            num_dimensions: NUM_DIMENSIONS,
            delta_t: DELTA_T,
            control_cost_weight: 0.0,
            initialization_method: TrajectoryInitialization::LinearInterpolation,
            exponentiated_cost_sensitivity: 0.5,
            num_rollouts: 20,
            max_rollouts: 20,
        }
    }

    #[test]
    fn construction_does_not_panic() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias,
            &BIAS_THRESHOLD,
            &STD_DEV,
            1,
        ));
        let _stomp = Stomp::new(create_3dof_configuration(NUM_TIMESTEPS), task);
    }

    #[test]
    fn solve_default_converges_to_the_bias_trajectory_from_endpoints() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            1,
        ));
        let mut stomp = Stomp::new(create_3dof_configuration(NUM_TIMESTEPS), task);

        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), NUM_TIMESTEPS);
        assert!(
            compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD),
            "optimized trajectory did not converge within BIAS_THRESHOLD of the bias trajectory"
        );
    }

    #[test]
    fn solve_with_linear_interpolated_initial_trajectory_converges() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            2,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.initialization_method = TrajectoryInitialization::LinearInterpolation;
        let mut stomp = Stomp::new(config, task);

        let (_, optimized) = stomp.solve(&trajectory_bias);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), NUM_TIMESTEPS);
        assert!(compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD));
    }

    #[test]
    fn solve_with_cubic_polynomial_initial_trajectory_converges() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            3,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.initialization_method = TrajectoryInitialization::CubicPolynomialInterpolation;
        let mut stomp = Stomp::new(config, task);

        let (_, optimized) = stomp.solve(&trajectory_bias);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), NUM_TIMESTEPS);
        assert!(compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD));
    }

    #[test]
    fn solve_with_minimum_control_cost_initial_trajectory_converges() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            4,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.initialization_method = TrajectoryInitialization::MinimumControlCost;
        let mut stomp = Stomp::new(config, task);

        let (_, optimized) = stomp.solve(&trajectory_bias);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), NUM_TIMESTEPS);
        assert!(compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD));
    }

    #[test]
    fn solve_with_40_timesteps_converges() {
        let num_timesteps = 40;
        let trajectory_bias = interpolate(&START_POS, &END_POS, num_timesteps);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            5,
        ));
        let mut config = create_3dof_configuration(num_timesteps);
        config.initialization_method = TrajectoryInitialization::LinearInterpolation;
        config.num_iterations = 100;
        let mut stomp = Stomp::new(config, task);

        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), num_timesteps);
        assert!(compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD));
    }

    #[test]
    fn solve_with_60_timesteps_converges() {
        let num_timesteps = 60;
        let trajectory_bias = interpolate(&START_POS, &END_POS, num_timesteps);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            6,
        ));
        let mut config = create_3dof_configuration(num_timesteps);
        config.num_iterations = 100;
        let mut stomp = Stomp::new(config, task);

        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), num_timesteps);
        assert!(compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD));
    }

    /// Not from upstream's own suite: a boundary test for
    /// [`CancelHandle`], upstream's `Stomp::cancel()` thread-safety
    /// contract this port had to restructure into a separate handle type
    /// (see this module's own doc). Cancels *before* `solve` is even
    /// called, matching the `proceed_` check at the very top of
    /// `runSingleIteration`/inside `generateNoisyRollouts`'s rollout loop
    /// -- the boundary is "cancellation observed on the very first check",
    /// not a race with a background thread (which a deterministic unit
    /// test cannot assert on).
    #[test]
    fn cancelling_before_solve_stops_before_num_iterations_completes() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias,
            &BIAS_THRESHOLD,
            &STD_DEV,
            7,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.num_iterations = 1_000_000;
        let mut stomp = Stomp::new(config, task);
        let cancel = stomp.cancel_handle();
        cancel.cancel();

        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), NUM_TIMESTEPS);
    }

    /// Invariant-boundary test for `Stomp::solve`'s seed-ignoring quirk
    /// (see `Stomp::solve`'s own doc): a second `solve` call on the same
    /// `Stomp` does not reseed from its `initial_parameters` argument once
    /// `parameters_optimized` is already non-zero from the first call.
    #[test]
    fn second_solve_call_ignores_its_initial_parameters_argument() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            8,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.num_iterations = 1;
        let mut stomp = Stomp::new(config, task);

        let (_, first) = stomp.solve(&trajectory_bias);
        // A wildly different "initial" trajectory: if solve() actually
        // reseeded from it, `second` would start optimizing from an
        // all-100s matrix instead of continuing from `first`.
        let bogus_initial = DMatrix::from_element(NUM_DIMENSIONS, NUM_TIMESTEPS, 100.0);
        let (_, second) = stomp.solve(&bogus_initial);

        assert_ne!(
            second,
            DMatrix::from_element(NUM_DIMENSIONS, NUM_TIMESTEPS, 100.0)
        );
        let _ = first;
    }
}
