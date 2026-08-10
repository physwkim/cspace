// Copyright (c) 2016, Southwest Research Institute
// Copyright (c) 2026, cspace contributors
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
//! `cspace_core::sampling::MultivariateGaussian::sample_with_covariance` (per
//! this round's brief: STOMP is the covariance-using caller of that class)
//! seeded by a fixed `rand_chacha::ChaCha8Rng`, so the test is
//! deterministic without claiming to reproduce upstream's specific libc
//! PRNG stream. The assertion this port keeps from upstream is the
//! meaningful one: `solve()` converges to within `BIAS_THRESHOLD` of the
//! bias trajectory it was scored against.
//!
//! Round 26 re-verified this against `test/stomp_3dof.cpp` line-by-line
//! rather than taking the paragraph above on faith: all 7 of upstream's
//! `TEST(Stomp3DOF, ...)` cases (`construction`, `solve_default`,
//! `solve_interpolated_initial`, `solve_cubic_polynomial_initial`,
//! `solve_min_control_cost_initial`, `solve_40_timesteps`,
//! `solve_60_timesteps`) have a 1:1 counterpart below, and every numeric
//! constant this port's tests use (`NUM_DIMENSIONS`, `NUM_TIMESTEPS`,
//! `DELTA_T`, `START_POS`, `END_POS`, `BIAS_THRESHOLD`, `STD_DEV`,
//! `num_iterations`, `num_rollouts`, `max_rollouts`,
//! `num_iterations_after_valid`, `control_cost_weight`, and each
//! per-scenario `num_timesteps`/`num_iterations` override) matches
//! `test/stomp_3dof.cpp`'s own literals exactly. This closes item 2 of
//! round 26's brief -- porting upstream's own acceptance test -- which had
//! already substantially happened in an earlier round; this round's
//! contribution is the line-by-line re-verification above plus one real,
//! previously-undocumented finding from it: `compare_diff`'s own doc
//! comment below, "upstream's own `cwiseAbs()` call is dead code".
//!
//! # Completeness audit (round 26): `stomp.h` + `stomp.cpp`
//!
//! `stomp.h`'s `class Stomp` has 7 `public:` members (the constructor plus 6
//! methods), 11 `protected:` methods, and 21 `protected:` data members;
//! `stomp.cpp` additionally defines 7 file-local symbols (3 constants, 2
//! `static` helpers, and 2 non-`static`-but-header-undeclared free
//! functions) not declared in any header. "public 심볼만" would leave
//! `Stomp`'s entire computational core (all 11 protected methods) and every
//! `.cpp`-local helper unaudited, so this walk covers all three tiers —
//! `public:`, `protected:`, and file-local — explicitly, not just
//! `public:`. `protected:` has no meaningful Rust translation here (`Stomp`
//! is not subclassed anywhere in this port, matching D4: no virtual
//! inheritance), so every protected method is a private (non-`pub`) `fn`
//! below; that access-level narrowing is not called out per-symbol.
//!
//! Public (7, plus the class itself = 8):
//! - `class Stomp` — ported as [`Stomp<'a>`]; the `'a` lifetime is a round-23
//!   addition with no upstream equivalent (`TaskPtr` is a `shared_ptr` with
//!   no borrow to track) — see this type's own doc.
//! - `Stomp(config, task)` — ported as [`Stomp::new`].
//! - `solve(std::vector<double>, std::vector<double>, Eigen::MatrixXd&)` —
//!   ported as [`Stomp::solve_from_endpoints`].
//! - `solve(Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd&)` — distinct:
//!   upstream needs a second overload only because `std::vector<double>` and
//!   `Eigen::VectorXd` are unrelated container types; `&[f64]` covers both
//!   call shapes in Rust, so [`Stomp::solve_from_endpoints`] is this
//!   overload too, not a separate symbol.
//! - `solve(Eigen::MatrixXd, Eigen::MatrixXd&)` — ported as [`Stomp::solve`].
//! - `setConfig` — ported as [`Stomp::set_config`].
//! - `cancel` — ported as [`Stomp::cancel`]; see this module's own doc for
//!   why [`CancelHandle`] exists alongside it.
//! - `clear` — ported as [`Stomp::clear`].
//!
//! Protected (11):
//! - `resetVariables` — ported as `Stomp::reset_variables`.
//! - `computeInitialTrajectory` — ported as `Stomp::compute_initial_trajectory`.
//! - `runSingleIteration` — ported as `Stomp::run_single_iteration`.
//! - `generateNoisyRollouts` — ported as `Stomp::generate_noisy_rollouts`.
//! - `filterNoisyRollouts` — ported as `Stomp::filter_noisy_rollouts`.
//! - `computeNoisyRolloutsCosts` — ported as `Stomp::compute_noisy_rollouts_costs`.
//! - `computeRolloutsStateCosts` — ported as `Stomp::compute_rollouts_state_costs`.
//! - `computeRolloutsControlCosts` — ported as `Stomp::compute_rollouts_control_costs`.
//! - `computeProbabilities` — ported as `Stomp::compute_probabilities`.
//! - `updateParameters` — ported as `Stomp::update_parameters`.
//! - `computeOptimizedCost` — ported as `Stomp::compute_optimized_cost`.
//!
//! Protected data members (21, `rg -c '_;\s*(/\*\*<)?' stomp.h` restricted to
//! the member block confirms 21):
//! - `proceed_` — ported as `Stomp::proceed`; type widened from
//!   `std::atomic<bool>` to `Arc<AtomicBool>` so [`CancelHandle`] can share
//!   it across threads (see this module's own doc).
//! - `task_` — ported as `Stomp::task` (`Box<dyn Task + 'a>`).
//! - `config_` — ported as `Stomp::config`.
//! - `current_iteration_` — ported as `Stomp::current_iteration`; stored as
//!   `i32` not `unsigned int` — every read of it flows into a `Task` method
//!   parameter typed `iteration_number: int` (`task.h`, signed) at every
//!   call site, so this port stores the type its data actually flows into
//!   rather than casting at each of those calls.
//! - `parameters_valid_` — ported as `Stomp::parameters_valid`.
//! - `parameters_valid_prev_` — ported as `Stomp::parameters_valid_prev`.
//! - `parameters_total_cost_` — ported as `Stomp::parameters_total_cost`.
//! - `current_lowest_cost_` — ported as `Stomp::current_lowest_cost`; seeded
//!   at construction with the value upstream only assigns inside `solve()`
//!   (upstream leaves it uninitialized until then) — see the field's own
//!   doc comment.
//! - `parameters_optimized_` — ported as `Stomp::parameters_optimized`.
//! - `parameters_updates_` — ported as `Stomp::parameters_updates`.
//! - `parameters_state_costs_` — ported as `Stomp::parameters_state_costs`.
//! - `parameters_control_costs_` — ported as `Stomp::parameters_control_costs`.
//! - `noisy_rollouts_` — ported as `Stomp::noisy_rollouts`.
//! - `reused_rollouts_` — ported as `Stomp::reused_rollouts`.
//! - `num_active_rollouts_` — ported as `Stomp::num_active_rollouts`.
//! - `num_timesteps_padded_` — distinct: `stomp.cpp:357` assigns it and
//!   `:359` reads it back immediately, both inside `resetVariables`; no
//!   other line in `stomp.cpp` ever reads it (confirmed by
//!   `rg -n 'num_timesteps_padded_' src/stomp.cpp`, 2 hits, both in
//!   `resetVariables`). This port keeps the computation but not the
//!   storage: a local `num_timesteps_padded` inside `reset_variables`
//!   (below), not a struct field — dead cross-method state in upstream,
//!   so nothing downstream can observe the difference.
//! - `start_index_padded_` — ported as `Stomp::start_index_padded`.
//! - `finite_diff_matrix_A_padded_` — ported as `Stomp::finite_diff_matrix_a_padded`.
//! - `control_cost_matrix_R_padded_` — ported as `Stomp::control_cost_matrix_r_padded`.
//! - `control_cost_matrix_R_` — ported as `Stomp::control_cost_matrix_r`.
//! - `inv_control_cost_matrix_R_` — ported as `Stomp::inv_control_cost_matrix_r`.
//!
//! `stomp.cpp` file-local, not declared in any header (7):
//! - `DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT` (`:36`) — ported as
//!   `utils::DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT`, relocated to `utils.rs`
//!   (its one consumer, `Rollout::new`, lives there) — see that constant's
//!   own doc comment.
//! - `MIN_COST_DIFFERENCE` (`:37`) — ported as `MIN_COST_DIFFERENCE` below
//!   (private, not linkable from this module-level doc -- see the constant
//!   itself just below this doc block).
//! - `MIN_CONTROL_COST_WEIGHT` (`:38`) — ported as `MIN_CONTROL_COST_WEIGHT`
//!   below, same as above.
//! - `computeLinearInterpolation` (`static`, `:47`) — ported as
//!   `compute_linear_interpolation` (private free function, below).
//! - `computeCubicInterpolation` (`static`, `:71`) — ported as
//!   `compute_cubic_interpolation` (private free function, below).
//! - `computeMinCostTrajectory` (non-`static`, global namespace,
//!   header-undeclared, `:103`) — ported as `compute_min_cost_trajectory`
//!   (private free function, below).
//! - `computeParametersControlCosts` (non-`static`, global namespace,
//!   header-undeclared, `:152`) — ported as `compute_parameters_control_costs`
//!   (private free function, below).
//!
//! Sum: 8 (public, incl. the class) + 11 (protected methods) + 21 (protected
//! data, 20 `ported as` + 1 `distinct`) + 7 (`stomp.cpp` file-local) = 47,
//! matching `stomp.h`'s 40 (`rg` above) + `stomp.cpp`'s 7 file-local symbols.
//! Zero `unported, in scope` and zero `D1 exclusion` in this file: every
//! upstream symbol in `stomp.h`/`stomp.cpp` has a Rust counterpart. Beyond
//! upstream, not counted in the 47 above: [`CancelHandle`] (struct + `new` +
//! `cancel` + `Default`) and [`Stomp::with_cancel_handle`]/
//! [`Stomp::cancel_handle`] — round 24's cancellation-handle split, no
//! upstream symbol to correspond to.

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
/// `Eigen::NumTraits<double>::dummy_precision()` -- the default tolerance
/// `Eigen::MatrixBase::isZero()` (and friends: `isApprox`, `isConstant`,
/// `isOnes`) uses when the caller does not pass an explicit precision.
/// `Stomp::solve` relies on this via `parameters_optimized_.isZero()`; see
/// that function's own doc for why an exact `== 0.0` check is not
/// equivalent. Verified against `Eigen/src/Core/NumTraits.h`'s
/// `NumTraits<double>::dummy_precision()`.
const EIGEN_DUMMY_PRECISION_F64: f64 = 1e-12;

/// A thread-safe handle to cancel an in-flight [`Stomp::solve`]. See this
/// module's own doc, "`Stomp::cancel()`'s thread-safety".
#[derive(Clone)]
pub struct CancelHandle(Arc<AtomicBool>);

impl CancelHandle {
    /// A fresh, not-yet-cancelled handle, obtainable *before* any [`Stomp`]
    /// exists. Round 24, not upstream: `Stomp::cancel_handle` (obtain
    /// *after* construction) is the only way upstream's `TaskPtr`/`Stomp`
    /// shape can expose one, since `proceed_` is `Stomp`'s own member. This
    /// port's [`Stomp::with_cancel_handle`] takes the reverse direction --
    /// a caller builds a handle first via this constructor, keeps a clone
    /// for itself (e.g. to hand to a timeout thread, matching upstream's
    /// `stomp_moveit_planning_context.cpp:247-257` watcher), and passes the
    /// original into `Stomp::with_cancel_handle` so the `Stomp` it
    /// constructs shares that same underlying flag instead of minting its
    /// own that nothing outside the constructor call can ever reach.
    pub fn new() -> Self {
        Self(Arc::new(AtomicBool::new(true)))
    }

    /// Requests cancellation. `solve()` checks this at the start of every
    /// iteration and, within `generate_noisy_rollouts`, before generating
    /// each noisy rollout -- the same points upstream's `proceed_` check
    /// gates.
    pub fn cancel(&self) {
        self.0.store(false, Ordering::SeqCst);
    }
}

impl Default for CancelHandle {
    fn default() -> Self {
        Self::new()
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
///
/// `None` when `num_timesteps < 2`, which would make `dtheta`'s denominator
/// zero and every column `NaN`. Upstream `computeLinearInterpolation`
/// (`stomp.cpp:47-60`) divides by the same `num_timesteps - 1` unguarded and
/// reports success. Signalled through the same `Option` channel
/// [`compute_min_cost_trajectory`] already uses.
fn compute_linear_interpolation(
    first: &[f64],
    last: &[f64],
    num_timesteps: usize,
) -> Option<DMatrix<f64>> {
    if num_timesteps < 2 {
        return None;
    }
    let mut trajectory_joints = DMatrix::zeros(first.len(), num_timesteps);
    for i in 0..first.len() {
        let dtheta = (last[i] - first[i]) / (num_timesteps as f64 - 1.0);
        for j in 0..num_timesteps {
            trajectory_joints[(i, j)] = first[i] + j as f64 * dtheta;
        }
    }
    Some(trajectory_joints)
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
///
/// `None` when `total_time` is not positive and finite -- `num_points < 2` or
/// a non-positive `dt` -- which would make every `c2`/`c3` coefficient
/// infinite and the whole trajectory `NaN`. Upstream
/// `computeCubicInterpolation` divides by the same `total_time` unguarded and
/// reports success. Same `Option` channel as
/// [`compute_linear_interpolation`].
fn compute_cubic_interpolation(
    first: &[f64],
    last: &[f64],
    num_points: usize,
    dt: f64,
) -> Option<DMatrix<f64>> {
    let total_time = (num_points as f64 - 1.0) * dt;
    if !total_time.is_finite() || total_time <= 0.0 {
        return None;
    }
    let mut trajectory_joints = DMatrix::zeros(first.len(), num_points);
    for i in 0..first.len() {
        let c0 = first[i];
        let c2 = (3.0 / total_time.powi(2)) * (last[i] - first[i]);
        let c3 = (-2.0 / total_time.powi(3)) * (last[i] - first[i]);
        for j in 0..num_points {
            let t = j as f64 * dt;
            trajectory_joints[(i, j)] = c0 + c2 * t.powi(2) + c3 * t.powi(3);
        }
    }
    Some(trajectory_joints)
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
/// `JointModelGroup` -- see `cspace_planners::stomp::filter_functions::
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

    /// Test-only regression hook, absent from release builds. See
    /// `tests::each_convergence_test_fails_if_the_parameter_update_is_disabled`
    /// for what it reproduces and why it exists as a permanent field rather
    /// than a temporary source edit.
    #[cfg(test)]
    disable_accept_update_for_test: bool,
}

impl<'a> Stomp<'a> {
    /// `Stomp(config, task)`. Mints its own fresh, unreachable-from-outside
    /// `proceed` flag -- see [`Stomp::with_cancel_handle`] for a
    /// caller-supplied one a second thread can hold onto and cancel while
    /// `solve()` is in flight.
    pub fn new(config: StompConfiguration, task: Box<dyn Task + 'a>) -> Self {
        Self::with_cancel_handle(config, task, CancelHandle::new())
    }

    /// `Stomp(config, task)`, sharing `cancel_handle`'s underlying flag
    /// instead of minting a private one. Round 24, not upstream: gives a
    /// caller a handle *before* this `Stomp` exists, obtained via
    /// [`CancelHandle::new`] -- see that constructor's own doc for why this
    /// is the direction `cspace_planners::stomp::planner::plan` needs
    /// (accept an already-built handle rather than build-and-immediately-
    /// discard one internally, which was this crate's round-23 UNFIXED
    /// gap).
    ///
    /// If `cancel_handle` was already cancelled before this call, the
    /// `Stomp` returned here is already-cancelled too: [`Self::solve`]
    /// exits before running any iteration. See
    /// `Stomp::reset_variables`'s own "Deviation: does not touch
    /// `proceed`" for the bug this guarantee once had (construction
    /// silently un-cancelled a pre-cancelled handle) and why it does not
    /// anymore.
    pub fn with_cancel_handle(
        config: StompConfiguration,
        task: Box<dyn Task + 'a>,
        cancel_handle: CancelHandle,
    ) -> Self {
        let mut stomp = Self {
            task,
            config,
            proceed: cancel_handle.0,
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
            #[cfg(test)]
            disable_accept_update_for_test: false,
        };
        stomp.reset_variables();
        stomp
    }

    /// `setConfig`: replaces the configuration and re-derives every
    /// internal matrix from it. Also un-cancels `proceed`, matching
    /// upstream's own `setConfig` -> `resetVariables` -> `proceed_ = true`
    /// (`stomp.cpp:176-180,289`): reconfiguring an existing `Stomp` for
    /// reuse is a deliberate restart, so any earlier cancellation --
    /// same-thread [`Stomp::cancel`] or a [`CancelHandle::cancel`] this
    /// `Stomp` shares -- is intentionally forgotten. See
    /// `Stomp::reset_variables`'s own doc for why the un-cancel is not
    /// inside `reset_variables` itself.
    pub fn set_config(&mut self, config: StompConfiguration) {
        self.config = config;
        self.proceed.store(true, Ordering::SeqCst);
        self.reset_variables();
    }

    /// `clear`: resets all internal variables without changing the
    /// configuration. Also un-cancels `proceed` -- see [`Stomp::set_config`]'s
    /// doc for why.
    pub fn clear(&mut self) {
        self.proceed.store(true, Ordering::SeqCst);
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
    /// optimizer when `parameters_optimized` is (Eigen-)zero
    ///
    /// Upstream: `if (parameters_optimized_.isZero()) { parameters_optimized_
    /// = initial_parameters; }`. `Eigen::MatrixBase::isZero()` is a
    /// per-element tolerance check, not exact equality --
    /// `Eigen/src/Core/CwiseNullaryOp.h`'s `isZero(prec)` requires
    /// `isMuchSmallerThan(coeff, 1, prec)` for every coefficient, which
    /// `Eigen/src/Core/MathFunctions.h`'s real-scalar `isMuchSmallerThan`
    /// defines as `abs(coeff) <= prec`, at the default `prec =
    /// EIGEN_DUMMY_PRECISION_F64` (`NumTraits<double>::dummy_precision()`,
    /// `1e-12`). This port previously translated it as `.iter().all(|&v| v
    /// == 0.0)` -- exact bitwise equality -- which is not equivalent: a
    /// `parameters_optimized` that has drifted to within `1e-12` of zero in
    /// every entry (plausible after real optimizer arithmetic converges
    /// toward an all-zero optimum) but is not bitwise `0.0` anywhere would
    /// read as "already seeded" here while upstream's `isZero()` would
    /// still treat it as unseeded and reseed from `initial_parameters`.
    /// Reproduced directly (not just read): `stomp::tests::
    /// solve_reseeds_from_near_zero_but_not_exactly_zero_state`.
    ///
    /// On a freshly constructed (or just-`clear`ed) [`Stomp`],
    /// `parameters_optimized` starts at all-zero either way, so
    /// `initial_parameters` does seed it. Calling `solve` a *second* time on
    /// the same `Stomp` without resetting leaves `parameters_optimized`
    /// non-(Eigen-)zero from the first call in the ordinary case, so
    /// `initial_parameters` is silently **not** used to seed the second run
    /// -- the previous run's result is reused as the starting point instead,
    /// though `initial_parameters`'s shape is still validated below either
    /// way. Preserved exactly, not "fixed": this is a real, if surprising,
    /// documented-by-observation upstream behavior, not a translation bug.
    ///
    /// Also preserved: upstream's dimension check is written as an outer
    /// `if`/`else` whose `else` branch re-checks a condition the outer `if`'s
    /// negation already rules out (`cols() != num_timesteps` cannot be true
    /// inside the `else` of `rows() != num_dimensions || cols() !=
    /// num_timesteps`) -- provably unreachable, so this port collapses it to
    /// the one reachable check rather than translating dead code.
    ///
    /// # Deviations from upstream
    ///
    /// **Returns `false` when the optimized trajectory is not finite, which
    /// upstream does not.** [`StompConfiguration`] is a plain `pub`-field
    /// struct here exactly as upstream's is, so there is no construction
    /// point to validate at, and neither upstream's `Stomp::solve`
    /// (`stomp.cpp:208`) nor its `generateSmoothingMatrix` (`utils.cpp:61`)
    /// checks `dt` before dividing by it. With `control_cost_weight`
    /// non-zero, `delta_t == 0.0` returned a fully-`NaN` trajectory
    /// alongside `true`, and no caller can tell that from a real solution:
    /// `parameters_valid` comes from the task's own cost callbacks, which
    /// screen with ordinary comparisons, and every comparison against `NaN`
    /// is false.
    ///
    /// The check is on the returned answer rather than on `delta_t` because
    /// no closed-form bound on `delta_t` is sufficient. `0.0` and `NaN`
    /// poison the divides directly; `f64::MIN_POSITIVE` survives them
    /// (`dt.powi(2)` merely underflows) but its reciprocal is infinite; and
    /// `1e-150` survives *both* — reciprocal `1e300`, finite — only to
    /// overflow later inside `delta_t * A^T * A`. Each threshold that
    /// catches one of those admits the next, which is the shape of an edge
    /// factory. What `solve` promises is a usable trajectory, so that is
    /// what it verifies, and the rule then holds for numeric routes no
    /// config field explains.
    ///
    /// On this path the returned matrix is the caller's own
    /// `initial_parameters`, not the poisoned one, so a caller that ignores
    /// the flag still gets a finite trajectory. A `delta_t` past roughly
    /// `f64::MAX.sqrt()` reaches this same check: it makes
    /// `Stomp::reset_variables`' control-cost matrix singular, and that
    /// function returns a `NaN` inverse (Eigen's own behaviour) rather than
    /// aborting, so the rejection happens here too.
    pub fn solve(&mut self, initial_parameters: &DMatrix<f64>) -> (bool, DMatrix<f64>) {
        if self
            .parameters_optimized
            .iter()
            .all(|&v| v.abs() <= EIGEN_DUMMY_PRECISION_F64)
        {
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

        // `parameters_valid` alone is not enough to call this a solution.
        // It is set from the task's own cost/validity callbacks, every one
        // of which screens with an ordinary comparison, and every
        // comparison against `NaN` is false -- so a trajectory that is
        // entirely `NaN` sails through them and is returned as a success.
        // The reachable cause is `config.delta_t`: the control-cost path
        // divides by `dt` and by `dt.powi(2)`
        // (`generate_finite_difference_matrix`, `differentiate`,
        // `compute_parameters_control_costs`), so `0.0` and `NaN` poison it
        // directly, and `1e-150` does too by a longer route -- the
        // reciprocal is finite there, but `delta_t * A^T * A` then squares
        // entries near `1e300` into infinity. That last one is why this is
        // a check on the answer rather than a threshold on `delta_t`: no
        // closed-form bound on `dt` catches the cases that only overflow
        // later, inside a matrix product. Checking what `solve` actually
        // promises holds for every numeric route into a non-finite result,
        // including ones no `StompConfiguration` field explains.
        // Handing back the caller's own seed, not the poisoned matrix, so a
        // caller that ignores the flag still gets a finite trajectory.
        if parameters_optimized.iter().any(|v| !v.is_finite()) {
            return (false, initial_parameters.clone());
        }

        (self.parameters_valid, parameters_optimized)
    }

    /// `resetVariables`.
    ///
    /// # Deviation: does not touch `proceed`
    ///
    /// Upstream's `resetVariables` unconditionally sets `proceed_ = true`
    /// (`stomp.cpp:289`), and every upstream caller -- the constructor,
    /// `clear`, `setConfig` -- is fine with that because `proceed_` is a
    /// private member no other code can set before those calls run. This
    /// port added [`Stomp::with_cancel_handle`] (round 24, not upstream),
    /// which lets a caller cancel a [`CancelHandle`] *before* the `Stomp`
    /// that shares its flag is even constructed -- and
    /// `with_cancel_handle`'s constructor calls this function too. An
    /// unconditional `proceed = true` here silently un-cancels that
    /// caller-supplied flag the moment construction finishes, defeating the
    /// one thing `with_cancel_handle` exists to allow (found by mutation-
    /// testing `cancelling_before_plan_is_called_returns_the_unmodified_linear_interpolation_seed`
    /// in `cspace_planners::stomp::planner`: even *without* any mutation,
    /// cancelling before `plan()` still let a full iteration run, because
    /// this line reset the very flag the test had just cancelled). Callers
    /// that genuinely want a fresh, uncancelled `proceed` -- [`Stomp::clear`],
    /// [`Stomp::set_config`] -- now set it explicitly themselves, matching
    /// upstream's actual intent (an existing `Stomp` object being
    /// deliberately restarted) without stomping on a handle's
    /// pre-construction state that upstream never had a way to set in the
    /// first place.
    fn reset_variables(&mut self) {
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
        // Upstream's `num_timesteps_padded_` is a struct field, but this is
        // its only write and only read (`stomp.cpp:357,359`) -- see this
        // module's own completeness-audit doc for `num_timesteps_padded_`.
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
        // Upstream is `inv_control_cost_matrix_R_ =
        // control_cost_matrix_R_.fullPivLu().inverse()`, and Eigen's
        // `inverse()` on a singular matrix returns a non-finite matrix
        // rather than throwing -- so panicking here would be a
        // port-introduced divergence, not a preserved one. The premise the
        // old `expect` asserted is falsifiable from public API: a
        // `config.delta_t` past roughly `f64::MAX.sqrt()` underflows
        // `delta_t * A^T * A` to the zero matrix, which has no inverse.
        // `NaN` is what "no inverse" means numerically and is what the rest
        // of this function would compute from a non-finite Eigen result
        // anyway; `Stomp::solve`'s finiteness check then turns it into a
        // `false` return instead of an abort.
        self.inv_control_cost_matrix_r = full_piv_lu_try_inverse_or_empty(
            self.control_cost_matrix_r.clone(),
        )
        .unwrap_or_else(|| {
            DMatrix::from_element(
                self.control_cost_matrix_r.nrows(),
                self.control_cost_matrix_r.ncols(),
                f64::NAN,
            )
        });

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
                match compute_cubic_interpolation(
                    first,
                    last,
                    self.config.num_timesteps,
                    self.config.delta_t,
                ) {
                    Some(trajectory) => {
                        self.parameters_optimized = trajectory;
                        true
                    }
                    None => false,
                }
            }
            TrajectoryInitialization::LinearInterpolation => {
                match compute_linear_interpolation(first, last, self.config.num_timesteps) {
                    Some(trajectory) => {
                        self.parameters_optimized = trajectory;
                        true
                    }
                    None => false,
                }
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

        #[cfg(test)]
        if self.disable_accept_update_for_test {
            return true;
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
    use cspace_core::sampling::MultivariateGaussian;
    use nalgebra::DVector;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;
    use std::sync::atomic::AtomicUsize;

    /// Both initial-trajectory interpolators divide by `num_timesteps - 1`
    /// (cubic by way of `total_time = (num_timesteps - 1) * delta_t`), so a
    /// single-timestep config produced an all-`NaN` trajectory while
    /// `compute_initial_trajectory` still reported success. Boundaries, not
    /// scenarios: one case per way the divisor reaches zero, plus the
    /// smallest accepted value on each axis.
    #[test]
    fn a_degenerate_initial_trajectory_is_rejected_not_returned_as_nan() {
        let first = [0.0, 1.0];
        let last = [1.0, 1.0];

        assert!(compute_linear_interpolation(&first, &last, 1).is_none());
        assert!(compute_linear_interpolation(&first, &last, 0).is_none());
        assert!(compute_cubic_interpolation(&first, &last, 1, 0.1).is_none());
        assert!(compute_cubic_interpolation(&first, &last, 10, 0.0).is_none());
        assert!(compute_cubic_interpolation(&first, &last, 10, -0.1).is_none());

        let linear =
            compute_linear_interpolation(&first, &last, 2).expect("two timesteps is valid");
        assert!(linear.iter().all(|v| v.is_finite()));
        let cubic =
            compute_cubic_interpolation(&first, &last, 2, 0.1).expect("two timesteps is valid");
        assert!(cubic.iter().all(|v| v.is_finite()));
    }

    const NUM_DIMENSIONS: usize = 3;
    const NUM_TIMESTEPS: usize = 20;
    const DELTA_T: f64 = 0.1;
    const START_POS: [f64; 3] = [1.4, 1.4, 0.5];
    const END_POS: [f64; 3] = [-1.25, 1.0, -0.26];
    /// Upstream's own literal (`test/stomp_3dof.cpp:39`,
    /// `const std::vector<double> BIAS_THRESHOLD = { 0.050, 0.050, 0.050 };`)
    /// -- not tightened here even though this port's own deterministic
    /// `ChaCha8Rng` draws converge far inside it. Measured `compare_diff`'s
    /// `max_abs_diff` (round: margin audit) for each of the six
    /// `solve_*_converges` tests below against this `0.05` threshold:
    /// `solve_default_converges_to_the_bias_trajectory_from_endpoints` ~=
    /// 1.11e-16 (~4.5e14x), `solve_with_linear_interpolated_initial_trajectory_converges`
    /// ~= 2.78e-17 (~1.8e15x), `solve_with_cubic_polynomial_initial_trajectory_converges`
    /// ~= 2.22e-16 (~2.25e14x), `solve_with_minimum_control_cost_initial_trajectory_converges`
    /// ~= 5.55e-17 (~9.0e14x), `solve_with_40_timesteps_converges` ~=
    /// 5.55e-17 (~9.0e14x), `solve_with_60_timesteps_converges` ~= 5.55e-17
    /// (~9.0e14x) -- all effectively floating-point-noise-level convergence,
    /// not a meaningfully close call against `BIAS_THRESHOLD`. Contrast
    /// `solve_with_60_timesteps_converges_is_a_known_gap_in_this_probe`
    /// (`each_convergence_test_fails_if_the_accept_path_update_is_disabled`'s
    /// doc), whose own `max_abs_diff` measured ~= 0.0317 against the same
    /// `0.05` -- only ~1.58x, a genuinely tight case that this same probe's
    /// doc already documents as a known gap; see that test's own doc for a
    /// fragility read on this specific margin.
    ///
    /// # Reclassified (round: margin audit follow-up): smoke test, not a bound
    ///
    /// The astronomical margins above are not a loose-but-real bound, the
    /// way `0.681`/`PENALTY` in `cost_functions.rs` turned out to be (see
    /// that test's own doc for the fraction reading that gave it power) --
    /// there is no hidden tighter reading of the same quantity to add here,
    /// because the six tests below never exercise a multi-iteration search
    /// in the first place. Measured directly
    /// (`stomp.current_iteration` after `solve`/`solve_from_endpoints`
    /// returns, all four `initialization_method` variants, `#[cfg(test)]`
    /// probe run and reverted this round): every one of them runs **exactly
    /// one** real iteration, `current_iteration == 1`. Root cause:
    /// `create_3dof_configuration`'s `num_iterations_after_valid: 0` --
    /// upstream's own literal (`test/stomp_3dof.cpp:208`, used by all six of
    /// upstream's own equivalent tests too, not a port invention) -- breaks
    /// `Stomp::solve`'s loop as soon as one valid iteration completes, and
    /// every one of these six starts `parameters_optimized` at or
    /// bit-adjacent to `trajectory_bias` itself (either `solve(&trajectory_bias)`
    /// seeding it directly via `solve`'s zero-check shortcut -- itself
    /// reproducing upstream's own `Stomp::solve(const Eigen::MatrixXd&,
    /// ...)` overload, `src/stomp.cpp:208-215`, so `initialization_method`
    /// is dead in three of these six tests in upstream too, not a port bug
    /// -- or `solve_from_endpoints`'s `compute_linear_interpolation` landing
    /// within float-roundoff of the test's own independently-implemented
    /// `interpolate`). Starting at/near the cost function's own minimum
    /// means the single real iteration's noisy update almost always gets
    /// rejected regardless of its content, and the reject path's `-=`
    /// against the same `+=`'d value round-trips `parameters_optimized`
    /// back to within floating-point noise of where it started -- which is
    /// exactly what the ~1e-16-scale measurements above are: round-trip
    /// float noise, not search convergence.
    ///
    /// Confirmed with a targeted mutation distinct from the existing
    /// `each_convergence_test_fails_if_the_accept_path_update_is_disabled`
    /// probe (a `#[cfg(test)]`-only flag): flipped the sign of
    /// `compute_probabilities`'s softmax exponent (`-h * ...` to `h * ...`,
    /// `stomp.rs`'s own `compute_probabilities`) -- a realistic
    /// production-code bug that inverts which rollouts the optimizer
    /// prefers, not a test-only escape hatch. Run and reverted this round
    /// (`git diff` confirmed clean before committing): of the 84 tests
    /// across both crates, exactly **one** reddened --
    /// `cspace_planners::stomp::planner::tests::plan_finds_a_lower_cost_trajectory_than_the_initial_straight_line_through_an_obstacle`.
    /// None of the six `solve_*_converges` tests here caught it. A bug that
    /// inverts the optimizer's core search preference is invisible to every
    /// one of these six -- the strongest evidence that they are smoke tests
    /// ("does `solve` run and return something plausible") rather than
    /// convergence bounds, and, per the standing rule against tightening
    /// upstream-ported values/configs, not something this port can fix by
    /// tightening `BIAS_THRESHOLD` or `num_iterations_after_valid` without
    /// deviating from upstream's own test design.
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
        rollout_call_count: Arc<AtomicUsize>,
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
                rollout_call_count: Arc::new(AtomicUsize::new(0)),
            }
        }

        /// A handle onto this task's per-rollout call counter, incremented
        /// once per `compute_noisy_costs` call (one call per rollout, see
        /// `run_single_iteration`). Must be cloned off before the task is
        /// moved into `Box<dyn Task>`/`Stomp::new`, mirroring
        /// `cspace_planners::stomp::planner`'s `call_count`/`AtomicUsize`
        /// cancellation-detection pattern -- direct-equality assertions on
        /// solved output cannot distinguish "cancellation stopped the
        /// solver" from "the solver's own early-break logic stopped it for
        /// an unrelated reason", but a near-zero rollout call count can.
        fn rollout_call_count_handle(&self) -> Arc<AtomicUsize> {
            Arc::clone(&self.rollout_call_count)
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
            self.rollout_call_count.fetch_add(1, Ordering::SeqCst);
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
    ///
    /// # Round 26, found while re-verifying against upstream: upstream's own
    /// `cwiseAbs()` call is dead code; this port is two-sided on purpose
    ///
    /// `test/stomp_3dof.cpp:184-186`:
    /// ```cpp
    /// diff.row(d) = optimized.row(d) - desired.row(d);
    /// diff.row(d).cwiseAbs();
    /// if ((diff.row(d).array() > thresholds[d]).any())
    /// ```
    /// `Eigen::MatrixBase::cwiseAbs()` returns a new expression; called as a
    /// standalone statement with no assignment back into `diff`, its result
    /// is discarded. So upstream's actual check is one-sided --
    /// `optimized - desired > threshold` on the *signed* difference -- not
    /// `|optimized - desired| > threshold` as the function's own doc comment
    /// ("Compares whether two trajectories are close to each other") and
    /// the dead `cwiseAbs()` call both signal was intended. A trajectory
    /// that undershoots `desired` by an arbitrarily large negative amount
    /// passes upstream's literal C++. This port takes the absolute value
    /// before comparing -- the two-sided check the function's own name and
    /// doc comment describe -- rather than reproducing a no-op method call
    /// upstream itself does not appear to have intended (no comment nearby
    /// explains a deliberate one-sided check, and no other STOMP quirk this
    /// crate preserves is *test-harness-only* dead code rather than
    /// optimizer behavior). If upstream's literal one-sided behavior turns
    /// out to be load-bearing for some case this port's own tests don't
    /// exercise, that would be a reason to revisit this, not a reason to
    /// have guessed it silently either way.
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

    /// Each `delta_t` that drives the optimized trajectory non-finite by a
    /// *different* numeric route, plus a demonstrated opposite.
    ///
    /// `control_cost_weight` must be non-zero to exercise any of this:
    /// [`create_3dof_configuration`] leaves it `0.0`, which multiplies away
    /// the entire path `delta_t` feeds, so `delta_t = 0.0` measured `0/60`
    /// non-finite cells under the default config and `60/60` with the
    /// control cost enabled.
    ///
    /// `-0.1` is deliberately **not** here. It returns a wholly finite
    /// trajectory (`dt.powi(2)` is positive), so it is not a case of this
    /// defect; whether a negative control period should be rejected on
    /// physical grounds is a separate question, and upstream accepts it.
    #[test]
    fn solve_never_reports_success_with_a_non_finite_trajectory() {
        for (dt, route) in [
            (0.0, "1.0 / dt is infinite"),
            (f64::NAN, "dt poisons every product it enters"),
            (f64::INFINITY, "1.0 / dt is zero, so R is singular"),
            (f64::NEG_INFINITY, "same, through the negative end"),
            (f64::MIN_POSITIVE, "dt.powi(2) underflows to zero"),
            // Reciprocal *is* finite here (1e300): the overflow happens
            // later, inside `delta_t * A^T * A`. This is the case no
            // closed-form threshold on `delta_t` catches, and the reason
            // the check is on the answer rather than on the input.
            (1e-150, "A^T * A overflows on entries near 1e300"),
        ] {
            let (ok, optimized) = solve_with_delta_t(dt);
            assert!(!ok, "solve reported success for delta_t = {dt} ({route})");
            let non_finite = optimized.iter().filter(|v| !v.is_finite()).count();
            assert_eq!(
                non_finite,
                0,
                "solve rejected delta_t = {dt} but still handed back \
                 {non_finite}/{} non-finite cells instead of the seed",
                optimized.len()
            );
        }
        // The demonstrated opposite: a usable `delta_t` still succeeds, so
        // the assertions above are not passing by rejecting everything.
        let (ok, optimized) = solve_with_delta_t(DELTA_T);
        assert!(ok, "solve rejected the usable delta_t {DELTA_T}");
        assert!(optimized.iter().all(|v| v.is_finite()));
    }

    /// `Stomp::new` must not panic on a `StompConfiguration` a caller can
    /// legally build. Past roughly `f64::MAX.sqrt()`, `delta_t * A^T * A`
    /// underflows to the zero matrix, which has no inverse -- upstream's
    /// `fullPivLu().inverse()` returns a non-finite matrix there rather
    /// than throwing, so a panic is a port-introduced divergence.
    #[test]
    fn a_delta_t_that_makes_the_control_cost_matrix_singular_is_rejected_not_a_panic() {
        for dt in [1e150, 1e200, f64::MAX] {
            let (ok, optimized) = solve_with_delta_t(dt);
            assert!(!ok, "solve reported success for delta_t = {dt}");
            assert!(
                optimized.iter().all(|v| v.is_finite()),
                "solve rejected delta_t = {dt} but handed back a non-finite trajectory"
            );
        }
    }

    fn solve_with_delta_t(delta_t: f64) -> (bool, DMatrix<f64>) {
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.delta_t = delta_t;
        // Non-zero so the control-cost path is actually reached; see the
        // caller's doc.
        config.control_cost_weight = 0.1;
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias,
            &BIAS_THRESHOLD,
            &STD_DEV,
            1,
        ));
        Stomp::new(config, task).solve_from_endpoints(&START_POS, &END_POS)
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

    /// Reproducible evidence for the claim (`doc/claim-audit/
    /// cspace-stomp-core.md`'s mutation-probe row) that the five
    /// `solve_*_converges` tests above genuinely assert on
    /// `update_parameters`'s accept-path accumulation
    /// (`self.parameters_optimized += &self.parameters_updates;`,
    /// `stomp.rs:1049` at the time this was written) rather than being
    /// satisfied by the seed alone -- i.e. that disabling that one line
    /// makes each of them fail. `Stomp::disable_accept_update_for_test`
    /// (`#[cfg(test)]`-only, absent from release builds) skips exactly
    /// that line; this is the same effect as commenting it out by hand,
    /// made permanent and re-runnable instead of a one-time source edit.
    ///
    /// This was found the manual way first: the coordinator disabled that
    /// line by hand and re-ran the crate's tests, and independently so did
    /// this worker. Both runs agreed on the same five failures reproduced
    /// here. Neither of us predicted `solve_with_40_timesteps_converges`
    /// would be one of the five -- its `num_iterations: 100` override
    /// suggested many iterations run, so a one-line accumulation change
    /// looked like it should be masked the same way `num_iterations_after_valid:
    /// 0` masked cancellation in earlier rounds. It was not: like every
    /// other test in this family, `solve`'s loop breaks after exactly one
    /// real iteration regardless of `num_iterations`, because the seed is
    /// already `parameters_valid` before the loop even starts (see
    /// `cancelling_before_solve_stops_before_num_iterations_completes`'s
    /// own doc, "this task's seed is already within `BIAS_THRESHOLD`").
    /// That one iteration still writes and then (on this reject-branch
    /// path) partially un-writes `parameters_optimized`, and it is that
    /// single iteration's arithmetic, not iteration count, that each of
    /// these five assertions actually depends on.
    ///
    /// This probe does **not** cover all 9 `create_3dof_configuration`
    /// call sites in this module. The other four:
    ///
    /// - `construction_does_not_panic`: never calls `solve`/`solve_from_endpoints`
    ///   at all -- there is no `compare_diff` assertion for this line to
    ///   threaten.
    /// - `cancelling_before_solve_stops_before_num_iterations_completes`:
    ///   raises `num_iterations_after_valid` itself and asserts on
    ///   `DummyTask`'s rollout call count, not on `compare_diff` -- already
    ///   independently regression-tested against a mutated `CancelHandle::cancel`,
    ///   not this line.
    /// - `second_solve_call_ignores_its_initial_parameters_argument`: asserts
    ///   `assert_ne!` between two `solve` outputs, not `compare_diff` against
    ///   a bias trajectory -- a different invariant.
    /// - `solve_with_60_timesteps_converges`: **is** a `compare_diff`
    ///   convergence assertion in the same family as the five covered here,
    ///   but this specific probe does not reliably fail it -- see
    ///   `solve_with_60_timesteps_converges_is_a_known_gap_in_this_probe`
    ///   immediately below for the measured reason and why this worker did
    ///   not force it into the covered set.
    #[test]
    fn each_convergence_test_fails_if_the_accept_path_update_is_disabled() {
        fn disabled(mut stomp: Stomp<'_>) -> Stomp<'_> {
            stomp.disable_accept_update_for_test = true;
            stomp
        }

        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            1,
        ));
        let mut stomp = disabled(Stomp::new(create_3dof_configuration(NUM_TIMESTEPS), task));
        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);
        assert!(
            !compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD),
            "solve_default_converges_to_the_bias_trajectory_from_endpoints' scenario still \
             converged with the accept-path update disabled"
        );

        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            2,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.initialization_method = TrajectoryInitialization::LinearInterpolation;
        let mut stomp = disabled(Stomp::new(config, task));
        let (_, optimized) = stomp.solve(&trajectory_bias);
        assert!(
            !compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD),
            "solve_with_linear_interpolated_initial_trajectory_converges' scenario still \
             converged with the accept-path update disabled"
        );

        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            3,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.initialization_method = TrajectoryInitialization::CubicPolynomialInterpolation;
        let mut stomp = disabled(Stomp::new(config, task));
        let (_, optimized) = stomp.solve(&trajectory_bias);
        assert!(
            !compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD),
            "solve_with_cubic_polynomial_initial_trajectory_converges' scenario still \
             converged with the accept-path update disabled"
        );

        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias.clone(),
            &BIAS_THRESHOLD,
            &STD_DEV,
            4,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.initialization_method = TrajectoryInitialization::MinimumControlCost;
        let mut stomp = disabled(Stomp::new(config, task));
        let (_, optimized) = stomp.solve(&trajectory_bias);
        assert!(
            !compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD),
            "solve_with_minimum_control_cost_initial_trajectory_converges' scenario still \
             converged with the accept-path update disabled"
        );

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
        let mut stomp = disabled(Stomp::new(config, task));
        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);
        assert!(
            !compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD),
            "solve_with_40_timesteps_converges' scenario still converged with the \
             accept-path update disabled"
        );
    }

    /// Companion to `each_convergence_test_fails_if_the_accept_path_update_is_disabled`,
    /// same probe applied to `solve_with_60_timesteps_converges`'s exact
    /// scenario (seed 6, 60 timesteps). Measured, not assumed: with
    /// `disable_accept_update_for_test` set, this scenario's single real
    /// iteration (see the sibling test's doc for why it is only ever one)
    /// still produces `compare_diff(&optimized, &trajectory_bias,
    /// &BIAS_THRESHOLD) == true` -- the assertion does not fail.
    ///
    /// Root cause, traced with `eprintln!` instrumentation on
    /// `update_parameters`/`compute_optimized_cost` (removed before this
    /// commit, not kept as permanent tracing): with the update disabled,
    /// `parameters_optimized` ends up shifted from the seed by exactly
    /// `parameters_updates` (the reject branch's `-=` still fires against
    /// the real, nonzero update; only the accept-path `+=` is skipped).
    /// The magnitude of that shift is this scenario's noise draw --
    /// seeded by `DummyTask::new`'s `seed: u64` and diluted across
    /// `num_timesteps` columns -- and for seed 6 / 60 timesteps that
    /// shift's per-element magnitude measured under `BIAS_THRESHOLD`'s
    /// 0.05, where for seed 5 / 40 timesteps
    /// (`solve_with_40_timesteps_converges`, covered above) it measured
    /// over it. Both are the same mechanism; only the fixed seed and
    /// column count differ, and those happen to land on opposite sides of
    /// the threshold.
    ///
    /// This is a genuine, honestly-reported gap, not a defect in
    /// `solve_with_60_timesteps_converges` the production test: that
    /// test's `compare_diff` assertion is real signal (it does depend on
    /// the seed actually landing close to the bias trajectory), it is just
    /// weak signal against *this one* mutation, for *this one* fixed seed.
    /// Picking a different seed to force a failure here would be
    /// p-hacking a coincidence, not fixing anything -- left open rather
    /// than papered over. See `doc/claim-audit/cspace-stomp-core.md` for
    /// the same statement recorded outside this file.
    ///
    /// # Fragility of the 1.58x margin (round: margin audit follow-up)
    ///
    /// Measured this round: `compare_diff`'s `max_abs_diff` here is ~=
    /// 0.0317 against `BIAS_THRESHOLD`'s `0.05` -- ~1.58x, the only
    /// genuinely tight margin among the seven `compare_diff` sites in this
    /// module (the other six are reclassified as smoke tests, no
    /// meaningful margin at all -- see `BIAS_THRESHOLD`'s own doc). That
    /// 1.58x is **not** margin against float rounding: this measurement's
    /// float noise floor is the ~1e-16-scale round-trip error the other six
    /// tests measure, twelve orders of magnitude below 0.0317, so ordinary
    /// arithmetic-order changes (a different summation order, a `nalgebra`
    /// point release) cannot move this number by anything close to 1.58x.
    /// What *can* move it by a full re-draw's worth -- easily clearing this
    /// margin in either direction -- is anything that shifts which values
    /// this scenario's `ChaCha8Rng` (seeded `6`) draws land on:
    /// adding/removing/reordering a call to `sample_with_covariance` per
    /// dimension or per rollout in `generate_noisy_rollouts`, changing
    /// `num_rollouts`/`max_rollouts` in `create_3dof_configuration`,
    /// changing `exponentiated_cost_sensitivity`, or an upstream
    /// `rand`/`rand_chacha` version bump that changes the byte stream for
    /// the same seed. None of those are float-precision drift; each is a
    /// discrete jump to a different draw, and this test's own root-cause
    /// paragraph above already traces the mechanism precisely enough to see
    /// that no continuity argument protects 1.58x against it. This margin
    /// is fragile: it is one otherwise-unremarkable refactor to
    /// `generate_noisy_rollouts`'s call order or rollout count away from
    /// silently flipping which side of `BIAS_THRESHOLD` this specific
    /// seed/timestep-count combination lands on -- this test currently
    /// asserts convergence *survives* the disabled update (the known gap:
    /// this one seed's shift happens to undershoot `0.05`), and a shift of
    /// its RNG draw could just as easily flip it to fail like its five
    /// siblings do, or a siblings' shift could flip to pass like this one
    /// does. Either direction is a silent behavior change in what this
    /// probe demonstrates, not a compile error or an obvious diff. A future
    /// round touching RNG call order or rollout counts in `stomp.rs` should
    /// re-measure this margin before trusting this test to still mean what
    /// it currently documents.
    #[test]
    fn solve_with_60_timesteps_converges_is_a_known_gap_in_this_probe() {
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
        stomp.disable_accept_update_for_test = true;
        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);
        assert!(
            compare_diff(&optimized, &trajectory_bias, &BIAS_THRESHOLD),
            "solve_with_60_timesteps_converges' scenario no longer converges with the \
             accept-path update disabled -- if this now fails, the known gap this test \
             pins has closed on its own (a different ChaCha8Rng draw, a changed \
             BIAS_THRESHOLD, or similar); update this test's doc and \
             doc/claim-audit/cspace-stomp-core.md to match rather than just relaxing the \
             assertion"
        );
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
    ///
    /// # Structural fix: dimensions-only assertion could not fail
    ///
    /// The original version of this test asserted only `optimized.nrows()`/
    /// `.ncols()` against the expected shape -- a shape `solve` returns
    /// regardless of how many iterations actually ran, cancelled or not, so
    /// the assertion could not have failed even with `CancelHandle::cancel`
    /// mutated to a no-op. Mutation-testing `CancelHandle::cancel` (empty
    /// body) confirmed this: the test still passed, in 0.012s, for a second,
    /// independent reason on top of that -- `create_3dof_configuration`'s
    /// `num_iterations_after_valid: 0` makes `Stomp::solve`'s own loop break
    /// after exactly one valid iteration regardless of `proceed`, since this
    /// task's seed is already within `BIAS_THRESHOLD`. Both gaps are closed
    /// here the same way `cancelling_from_another_thread_stops_a_plan_call_already_in_flight`
    /// (`cspace_planners::stomp::planner`) closes the identical pair: raise
    /// `num_iterations_after_valid` so the early-valid break cannot fire
    /// before `num_iterations` does, and assert on
    /// `DummyTask`'s per-rollout call count instead of on output shape, so
    /// the assertion actually depends on how much work `solve` did.
    ///
    /// # Tightened (round: margin audit): exact count, not an order-of-magnitude bound
    ///
    /// This test cancels on the calling thread *before* `solve_from_endpoints`
    /// is invoked at all -- unlike its sibling
    /// `cancelling_from_another_thread_stops_a_plan_call_already_in_flight`
    /// (`cspace_planners::stomp::planner`), there is no second thread and no
    /// race: `solve`'s pre-loop `compute_optimized_cost()` call
    /// (`Stomp::solve`'s own doc) always runs exactly once regardless of
    /// `proceed`, and `run_single_iteration` checks `proceed` before doing
    /// anything else, so the `while` loop body never executes once
    /// cancelled. `calls` is therefore deterministically `1`, not merely
    /// "orders of magnitude below" some uncancelled-run estimate -- measured
    /// at `calls=1` against the previous `calls * 1000 <
    /// plausible_uncancelled_calls` bound (`plausible_uncancelled_calls =
    /// 20_000_000`), a ~20,000x margin for a value with zero variance. That
    /// `* 1000` heuristic was this port's own invention (not from upstream,
    /// which has no equivalent test), copied from the genuinely racy sibling
    /// test above where a loose multiplier is the correct tool; here it hid
    /// the real invariant behind an unnecessarily wide bound. Asserting the
    /// exact count both tightens the check and states the invariant this
    /// test can actually prove: cancelling before `solve` is called permits
    /// precisely the one unconditional pre-loop cost evaluation and nothing
    /// else.
    #[test]
    fn cancelling_before_solve_stops_before_num_iterations_completes() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = DummyTask::new(trajectory_bias, &BIAS_THRESHOLD, &STD_DEV, 7);
        let rollout_call_count = task.rollout_call_count_handle();
        let task = Box::new(task);
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.num_iterations = 1_000_000;
        // See this test's own doc: without this, `solve`'s early-valid
        // break exits after one iteration on its own, masking whether
        // cancellation did anything.
        config.num_iterations_after_valid = config.num_iterations;
        let mut stomp = Stomp::new(config, task);
        let cancel = stomp.cancel_handle();
        cancel.cancel();

        let (_, optimized) = stomp.solve_from_endpoints(&START_POS, &END_POS);

        assert_eq!(optimized.nrows(), NUM_DIMENSIONS);
        assert_eq!(optimized.ncols(), NUM_TIMESTEPS);

        let calls = rollout_call_count.load(Ordering::SeqCst);
        assert_eq!(
            calls, 1,
            "DummyTask::compute_noisy_costs was called {calls} times; cancelling before \
             solve_from_endpoints is called permits exactly one call, from solve's \
             unconditional pre-loop compute_optimized_cost -- any other count means \
             cancellation was not observed at the expected point"
        );
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

    /// Regression test for the `Stomp::solve` seeding-check finding: upstream's
    /// `parameters_optimized_.isZero()` is a per-element `<= 1e-12` tolerance
    /// check (`Eigen::NumTraits<double>::dummy_precision()`, see
    /// [`EIGEN_DUMMY_PRECISION_F64`]'s own doc), not exact equality. Before
    /// the fix this round, `Stomp::solve` used `.iter().all(|&v| v ==
    /// 0.0)`, which failed exactly this case: `num_iterations: 0` isolates
    /// the seeding decision from any real iteration (the `while` loop body
    /// never runs, so `parameters_optimized` after `solve` returns is
    /// exactly whatever the seeding check produced), and `parameters_optimized`
    /// is set directly to an all-`1e-13` matrix -- within Eigen's `1e-12`
    /// tolerance of zero in every entry, but not bitwise `0.0` anywhere.
    #[test]
    fn solve_reseeds_from_near_zero_but_not_exactly_zero_state() {
        let trajectory_bias = interpolate(&START_POS, &END_POS, NUM_TIMESTEPS);
        let task = Box::new(DummyTask::new(
            trajectory_bias,
            &BIAS_THRESHOLD,
            &STD_DEV,
            1,
        ));
        let mut config = create_3dof_configuration(NUM_TIMESTEPS);
        config.num_iterations = 0;
        let mut stomp = Stomp::new(config, task);
        stomp.parameters_optimized = DMatrix::from_element(NUM_DIMENSIONS, NUM_TIMESTEPS, 1e-13);

        let seed = DMatrix::from_element(NUM_DIMENSIONS, NUM_TIMESTEPS, 100.0);
        let (_, optimized) = stomp.solve(&seed);

        assert_eq!(
            optimized, seed,
            "a parameters_optimized within Eigen's isZero() tolerance of zero (every entry \
             1e-13) but not bitwise 0.0 should still be treated as unseeded and reseeded from \
             initial_parameters, matching upstream's parameters_optimized_.isZero() check"
        );
    }
}
