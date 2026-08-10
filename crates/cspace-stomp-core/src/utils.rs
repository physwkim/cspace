// Copyright (c) 2016, Southwest Research Institute
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros-industrial/stomp @ b1a87c80f7338caae25a5c689b876da15492aa75:
//   include/stomp/utils.h
//   src/utils.cpp

//! # Completeness audit (round 26): `utils.h` + `utils.cpp`
//!
//! `utils.h` has 14 top-level symbols (2 structs, 2 enums, 3 constants, 7
//! free functions); struct fields and enum variants are not itemized as
//! separate bullets here, matching `cspace-scene`'s precedent of not
//! itemizing private/protected data members individually (both
//! [`Rollout`]'s 10 fields and [`StompConfiguration`]'s 10 fields are
//! confirmed 1:1 against `include/stomp/utils.h:38-58`/`:88-106` below, just
//! not bulleted one-by-one). `utils.cpp` adds nothing beyond `utils.h`'s own
//! declarations — confirmed by
//! `rg -n '^(static |bool |void |double |std::string )' src/utils.cpp`
//! matching only the 7 already-declared functions' definitions, no
//! additional file-local statics or helpers.
//!
//! - `struct Rollout` — ported as [`Rollout`]; all 10 fields present
//!   (`noise`, `parameters_noise`, `state_costs`, `control_costs`,
//!   `total_costs`, `probabilities`, `full_probabilities`, `full_costs`,
//!   `importance_weight`, `total_cost`).
//! - `DerivativeOrders::DerivativeOrder` (4 variants) — ported as
//!   [`DerivativeOrder`] (`Position`/`Velocity`/`Acceleration`/`Jerk`, same
//!   4 discriminants `0`-`3`).
//! - `TrajectoryInitializations::TrajectoryInitialization` (3 variants) —
//!   ported as [`TrajectoryInitialization`]
//!   (`LinearInterpolation`/`CubicPolynomialInterpolation`/`MinimumControlCost`,
//!   same 3 discriminants `1`-`3`).
//! - `struct StompConfiguration` — ported as [`StompConfiguration`]; all 10
//!   fields present (`num_iterations`, `num_iterations_after_valid`,
//!   `num_timesteps`, `num_dimensions`, `delta_t`, `initialization_method`,
//!   `exponentiated_cost_sensitivity`, `num_rollouts`, `max_rollouts`,
//!   `control_cost_weight`); `initialization_method` is the enum type
//!   itself here, not upstream's raw `int` discriminant — see the struct's
//!   own doc comment.
//! - `FINITE_DIFF_RULE_LENGTH` — ported as [`FINITE_DIFF_RULE_LENGTH`].
//! - `FINITE_CENTRAL_DIFF_COEFFS` — ported as [`FINITE_CENTRAL_DIFF_COEFFS`].
//! - `FINITE_FORWARD_DIFF_COEFFS` — ported as [`FINITE_FORWARD_DIFF_COEFFS`].
//! - `generateFiniteDifferenceMatrix` — ported as
//!   [`generate_finite_difference_matrix`] (returns `DMatrix<f64>` rather
//!   than writing through the `diff_matrix` out-parameter).
//! - `differentiate` — ported as [`differentiate`].
//! - `generateSmoothingMatrix` — ported as [`generate_smoothing_matrix`];
//!   upstream returns `void` unconditionally and assumes the smoothing
//!   matrix it builds is always invertible, but this port returns
//!   `Option<DMatrix<f64>>` and makes that assumption an explicit,
//!   checkable fallibility instead of an unstated one (see the function's
//!   own doc comment) — same computation, honest about its one partial
//!   case.
//! - `toVector` — ported as [`to_vector`].
//! - `toString(const std::vector<Eigen::VectorXd>&)` — ported as [`rows_to_string`].
//! - `toString(const Eigen::VectorXd&)` — ported as [`vector_to_string`].
//! - `toString(const Eigen::MatrixXd&)` — ported as [`matrix_to_string`].
//!
//! Sum: 2 (structs) + 2 (enums) + 3 (constants) + 7 (functions) = 14,
//! matching `rg -c '^(struct Rollout|enum DerivativeOrder|enum
//! TrajectoryInitialization|struct StompConfiguration|static const (int|
//! double) FINITE|void generateFiniteDifferenceMatrix|void differentiate\(|
//! void generateSmoothingMatrix|void toVector|std::string toString)'
//! include/stomp/utils.h` = 14. Zero `unported, in scope`, zero `D1
//! exclusion`. Beyond upstream, not counted in the 14 above:
//! [`Rollout::new`] (upstream's `Rollout` has no constructor),
//! [`full_piv_lu_try_inverse_or_empty`] (works around a `nalgebra`-specific
//! 0x0-matrix panic Eigen doesn't have — `pub`, not `pub(crate)`, because
//! `cspace_planners::stomp` needs the identical fix), and
//! `DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT` (upstream: `stomp.cpp`-file-local;
//! relocated here because [`Rollout::new`] is its one consumer — see that
//! constant's own doc comment, and `stomp.rs`'s completeness-audit doc for
//! the `stomp.cpp`-file-local accounting this relocation is subtracted
//! from).

use nalgebra::{DMatrix, DVector};

/// `TrajectoryInitializations::TrajectoryInitialization`. Upstream's
/// `StompConfiguration::initialization_method` stores this as a raw `int`
/// (there is no C++ enum class here, just a C-style `enum` implicitly
/// converted); this port uses the enum type itself in
/// [`StompConfiguration::initialization_method`] rather than an `int`
/// discriminant, so an invalid value cannot be constructed at all -- see
/// `stomp`'s module doc for how `Stomp::compute_initial_trajectory`
/// dispatches on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrajectoryInitialization {
    /// `LINEAR_INTERPOLATION = 1`.
    LinearInterpolation = 1,
    /// `CUBIC_POLYNOMIAL_INTERPOLATION`.
    CubicPolynomialInterpolation = 2,
    /// `MININUM_CONTROL_COST` (upstream's own spelling).
    MinimumControlCost = 3,
}

/// `StompConfiguration`. A plain data struct in upstream too (no
/// constructor, no default member values), so this port carries no
/// `Default` impl either; use struct-literal syntax.
///
/// Upstream's own `test/stomp_3dof.cpp::create3DOFConfiguration()` does
/// *not* actually set every field -- `exponentiated_cost_sensitivity` is
/// left untouched on a stack-allocated `StompConfiguration c;`, which C++
/// leaves as uninitialized memory (genuine UB, unconditionally read back as
/// `h` in both `Stomp::generateNoisyRollouts` and
/// `Stomp::computeProbabilities`). This port's own test
/// ([`crate::stomp`]'s test module) cannot reproduce that -- Rust has no
/// uninitialized-memory equivalent to assign -- and uses `0.5` instead,
/// `moveit2`'s own documented default for this field
/// (`moveit_planners/stomp/res/stomp_moveit.yaml`'s
/// `exponentiated_cost_sensitivity.default_value`), not a value read out of
/// `ros-industrial/stomp` itself.
#[derive(Debug, Clone, Copy)]
pub struct StompConfiguration {
    /// Maximum number of iterations allowed.
    pub num_iterations: usize,
    /// STOMP stops optimizing this many iterations after finding a valid
    /// solution.
    pub num_iterations_after_valid: usize,
    /// Number of timesteps.
    pub num_timesteps: usize,
    /// Parameter dimensionality.
    pub num_dimensions: usize,
    /// Time change between consecutive points.
    pub delta_t: f64,
    /// See [`TrajectoryInitialization`].
    pub initialization_method: TrajectoryInitialization,
    /// Default exponentiated cost sensitivity coefficient.
    pub exponentiated_cost_sensitivity: f64,
    /// Number of noisy trajectories generated per iteration.
    pub num_rollouts: usize,
    /// The combined number of new and reused rollouts during each iteration
    /// should not exceed this value.
    pub max_rollouts: usize,
    /// Percentage of the trajectory acceleration cost applied in the total
    /// cost calculation.
    pub control_cost_weight: f64,
}

/// `Rollout`: a single noisy trajectory sample and its costs.
///
/// Upstream's C-style struct leaves every field default-constructed with no
/// user-provided constructor; `total_cost` in particular is never assigned
/// a value in `Stomp::resetVariables`'s own template rollout (unlike
/// `importance_weight`, which upstream sets explicitly there) -- reading it
/// before `Stomp::computeNoisyRolloutsCosts` fills it in for real would be
/// reading uninitialized memory in C++. This port's [`Rollout::new`] sets it
/// to `0.0` instead, a harmless placeholder never observed before being
/// overwritten in any real code path, since every element of
/// `Stomp`'s `noisy_rollouts`/`reused_rollouts` is scanned across
/// `0..num_active_rollouts` -- always populated for real before that range
/// is ever read.
#[derive(Debug, Clone)]
pub struct Rollout {
    /// Random noise applied to the parameters, `[num_dimensions][num_timesteps]`.
    pub noise: DMatrix<f64>,
    /// Parameters + noise, `[num_dimensions][num_timesteps]`.
    pub parameters_noise: DMatrix<f64>,
    /// Cost at each timestep, `[num_timesteps]`.
    pub state_costs: DVector<f64>,
    /// Control cost for each parameter at every timestep,
    /// `[num_dimensions][num_timesteps]`.
    pub control_costs: DMatrix<f64>,
    /// `total_costs[d] = state_costs + control_costs[d]`,
    /// `[num_dimensions][num_timesteps]`.
    pub total_costs: DMatrix<f64>,
    /// Probability for each parameter at every timestep,
    /// `[num_dimensions][num_timesteps]`.
    pub probabilities: DMatrix<f64>,
    /// Probabilities for the full trajectory, one per dimension.
    pub full_probabilities: Vec<f64>,
    /// `full_costs[d] = state_costs.sum() + control_costs[d].sum()`, one per
    /// dimension.
    pub full_costs: Vec<f64>,
    /// Importance sampling weight.
    pub importance_weight: f64,
    /// Combined state + control cost over the entire trajectory, all
    /// dimensions. See this type's own doc for why it starts at `0.0`
    /// rather than mirroring an uninitialized C++ field.
    pub total_cost: f64,
}

impl Rollout {
    /// `Stomp::resetVariables`'s per-rollout template: every matrix sized
    /// `(num_dimensions, num_timesteps)` (`state_costs`: `num_timesteps`)
    /// and zeroed, `importance_weight` at upstream's
    /// `DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT`.
    pub fn new(num_dimensions: usize, num_timesteps: usize) -> Self {
        Self {
            noise: DMatrix::zeros(num_dimensions, num_timesteps),
            parameters_noise: DMatrix::zeros(num_dimensions, num_timesteps),
            state_costs: DVector::zeros(num_timesteps),
            control_costs: DMatrix::zeros(num_dimensions, num_timesteps),
            total_costs: DMatrix::zeros(num_dimensions, num_timesteps),
            probabilities: DMatrix::zeros(num_dimensions, num_timesteps),
            full_probabilities: vec![0.0; num_dimensions],
            full_costs: vec![0.0; num_dimensions],
            importance_weight: DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT,
            total_cost: 0.0,
        }
    }
}

/// `DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT` (`stomp.cpp`, file-local upstream;
/// [`Rollout::new`] is this port's one consumer, so it lives here rather
/// than in `stomp.rs`).
pub(crate) const DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT: f64 = 1.0;

/// Inverts a square control-cost-shaped matrix (`R = dt * A^T * A` for some
/// finite-difference `A`), sharing one fix between this module's
/// [`generate_smoothing_matrix`] and `stomp::Stomp::reset_variables`, which
/// independently builds the same shape of matrix upstream (`resetVariables`
/// computes its own `R`/`R^-1`, not by calling `generateSmoothingMatrix`).
/// `nalgebra`'s `FullPivLU::is_invertible` computes `nrows() - 1`
/// unconditionally, underflowing a `usize` and panicking for a 0x0 matrix,
/// where Eigen inverts an empty matrix without complaint -- see
/// [`generate_smoothing_matrix`]'s own "`num_timesteps == 0`" doc section
/// for the full reasoning.
///
/// `pub`, not `pub(crate)`: round 23 gave this a second consumer outside
/// this crate. `cspace_planners::stomp::noise_generators::
/// normal_distribution_generator` builds the exact same shape of matrix
/// (`getNormalDistributionGenerator`'s `acceleration.transpose() *
/// acceleration`, also `A^T * A` for a finite-difference `A`) and needs the
/// same 0x0-panic-safe inversion -- reusing this function keeps the fix in
/// its one place rather than re-deriving it a second time.
pub fn full_piv_lu_try_inverse_or_empty(m: DMatrix<f64>) -> Option<DMatrix<f64>> {
    if m.nrows() == 0 {
        return Some(DMatrix::zeros(0, 0));
    }
    m.full_piv_lu().try_inverse()
}

/// `DerivativeOrders::DerivativeOrder`. The discriminant doubles as both the
/// row index into [`FINITE_CENTRAL_DIFF_COEFFS`]/[`FINITE_FORWARD_DIFF_COEFFS`]
/// and the power of `dt` a finite-difference step divides by -- upstream
/// casts the enum straight to `int` for both uses (`(int)order` and
/// `FINITE_CENTRAL_DIFF_COEFFS[order]`), so the numeric values are load-bearing,
/// not just a Rust-side naming convenience.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DerivativeOrder {
    /// `STOMP_POSITION = 0`.
    Position = 0,
    /// `STOMP_VELOCITY = 1`.
    Velocity = 1,
    /// `STOMP_ACCELERATION = 2`.
    Acceleration = 2,
    /// `STOMP_JERK = 3`.
    Jerk = 3,
}

/// The number of columns in the finite differentiation rule.
pub const FINITE_DIFF_RULE_LENGTH: usize = 7;

/// Coefficients for finite *central* differentiation (position, velocity,
/// acceleration, jerk), one row per [`DerivativeOrder`] discriminant.
pub const FINITE_CENTRAL_DIFF_COEFFS: [[f64; FINITE_DIFF_RULE_LENGTH]; 4] = [
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [
        0.0,
        1.0 / 12.0,
        -2.0 / 3.0,
        0.0,
        2.0 / 3.0,
        -1.0 / 12.0,
        0.0,
    ],
    [
        0.0,
        -1.0 / 12.0,
        16.0 / 12.0,
        -30.0 / 12.0,
        16.0 / 12.0,
        -1.0 / 12.0,
        0.0,
    ],
    [
        0.0,
        1.0 / 12.0,
        -17.0 / 12.0,
        46.0 / 12.0,
        -46.0 / 12.0,
        17.0 / 12.0,
        -1.0 / 12.0,
    ],
];

/// Coefficients for finite *forward* differentiation (position, velocity,
/// acceleration, jerk), one row per [`DerivativeOrder`] discriminant.
pub const FINITE_FORWARD_DIFF_COEFFS: [[f64; FINITE_DIFF_RULE_LENGTH]; 4] = [
    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-25.0 / 12.0, 4.0, -3.0, 4.0 / 3.0, -1.0 / 4.0, 0.0, 0.0],
    [
        15.0 / 4.0,
        -77.0 / 6.0,
        107.0 / 6.0,
        -13.0,
        61.0 / 12.0,
        -5.0 / 6.0,
        0.0,
    ],
    [
        -49.0 / 8.0,
        29.0,
        -461.0 / 8.0,
        62.0,
        -307.0 / 8.0,
        13.0,
        -15.0 / 8.0,
    ],
];

/// `generateFiniteDifferenceMatrix`. Returns the matrix rather than writing
/// through an out-parameter -- see this crate's `lib.rs`, "Deviation:
/// return values, not out-parameters".
///
/// Near the first/last [`FINITE_DIFF_RULE_LENGTH`]`/2` rows, part of the
/// stencil falls outside `[0, num_time_steps)`. Upstream's loop reassigns
/// `index` to the clamped boundary (`0` or `num_time_steps - 1`) and then
/// immediately `continue`s without using that reassignment -- the clamp is
/// dead code, and the real effect is that the out-of-range coefficient is
/// simply dropped, not folded onto the boundary column. This port
/// reproduces the drop, not the (unreachable) clamp.
pub fn generate_finite_difference_matrix(
    num_time_steps: usize,
    order: DerivativeOrder,
    dt: f64,
) -> DMatrix<f64> {
    let mut diff_matrix = DMatrix::zeros(num_time_steps, num_time_steps);
    let multiplier = 1.0 / dt.powi(order as i32);
    let half = (FINITE_DIFF_RULE_LENGTH / 2) as isize;
    let n = num_time_steps as isize;
    for i in 0..n {
        for j in -half..=half {
            let index = i + j;
            if index < 0 || index >= n {
                continue;
            }
            diff_matrix[(i as usize, index as usize)] =
                multiplier * FINITE_CENTRAL_DIFF_COEFFS[order as usize][(j + half) as usize];
        }
    }
    diff_matrix
}

/// `generateSmoothingMatrix`. `None` where upstream's unchecked
/// `FullPivLU::inverse()` would silently return nonsense on a singular
/// `control_cost_matrix_R` -- see this crate's `lib.rs`, "Deviation:
/// `Option` where upstream doesn't check invertibility".
///
/// ```
/// use cspace_stomp_core::generate_smoothing_matrix;
///
/// let m = generate_smoothing_matrix(10, 0.1).expect("control_cost_matrix_R is invertible here");
/// assert_eq!((m.nrows(), m.ncols()), (10, 10));
/// // Exact by construction -- see this function's own tests for why.
/// assert!((m[(3, 3)] - 0.1).abs() < 1e-9);
/// ```
///
/// # `num_timesteps == 0`
///
/// Handled by `full_piv_lu_try_inverse_or_empty` before reaching
/// `full_piv_lu()`, not a case that flows through the general path.
/// Upstream's `int num_timesteps` lets Eigen invert a 0x0
/// `control_cost_matrix_R` without complaint (empty matrices are trivially
/// invertible, and its scaling loop's `for (t = 0; t < 0; t++)` never
/// runs). nalgebra's `FullPivLU::is_invertible` instead computes
/// `self.lu.nrows() - 1` unconditionally on a `usize`, which underflows for
/// a 0-dimension matrix and panics -- a `nalgebra` limitation on the 0x0
/// case, not a singular-`R` case this port's `Option` return is meant to
/// signal. Returning the empty matrix directly is the correct value for
/// this input either way, so it is special-cased rather than routed through
/// a library call that cannot express it.
pub fn generate_smoothing_matrix(num_timesteps: usize, dt: f64) -> Option<DMatrix<f64>> {
    let start_index_padded = FINITE_DIFF_RULE_LENGTH - 1;
    let num_timesteps_padded = num_timesteps + 2 * (FINITE_DIFF_RULE_LENGTH - 1);
    let finite_diff_matrix_a_padded =
        generate_finite_difference_matrix(num_timesteps_padded, DerivativeOrder::Acceleration, dt);

    // Upstream's own comment: "Original code multiplies the A product by
    // the time interval. However this is not what was described in the
    // literature" -- kept as upstream has it, not "corrected" to the
    // literature's version.
    let control_cost_matrix_r_padded =
        dt * finite_diff_matrix_a_padded.transpose() * &finite_diff_matrix_a_padded;
    let control_cost_matrix_r = control_cost_matrix_r_padded
        .view(
            (start_index_padded, start_index_padded),
            (num_timesteps, num_timesteps),
        )
        .into_owned();
    let mut projection_matrix_m = full_piv_lu_try_inverse_or_empty(control_cost_matrix_r)?;

    for t in 0..num_timesteps {
        let max = projection_matrix_m[(t, t)];
        let mut col = projection_matrix_m.column_mut(t);
        col *= 1.0 / (num_timesteps as f64 * max);
    }
    Some(projection_matrix_m)
}

/// `differentiate`. Returns the derivative vector rather than writing
/// through an out-parameter -- see this crate's `lib.rs`, "Deviation:
/// return values, not out-parameters".
///
/// Ported verbatim including one asymmetry with
/// [`generate_finite_difference_matrix`]: that function scales by
/// `1/dt^order`, but this function always divides by `dt^2` regardless of
/// `order` (`derivatives = A * parameters / std::pow(dt, 2)` in the
/// original, with no `(int)order` anywhere in the division). Whether that
/// is upstream's intended behavior or an unrelated inconsistency is not
/// this port's call to resolve; it is preserved exactly as read.
///
/// Upstream builds `A` by writing a length-[`FINITE_DIFF_RULE_LENGTH`]
/// coefficient segment into each row at an offset that depends on `i` and
/// `parameters.len()`; for a `parameters` shorter than the stencil needs,
/// the C++ segment write is undefined behavior. This port's indexing is
/// bounds-checked, so the same too-short input instead panics with a clear
/// message rather than corrupting memory.
pub fn differentiate(parameters: &DVector<f64>, order: DerivativeOrder, dt: f64) -> DVector<f64> {
    let central_coeffs = &FINITE_CENTRAL_DIFF_COEFFS[order as usize];
    let forward_coeffs = &FINITE_FORWARD_DIFF_COEFFS[order as usize];
    let mut backward_coeffs: Vec<f64> = forward_coeffs.iter().rev().copied().collect();
    if (order as i32) % 2 != 0 {
        for c in &mut backward_coeffs {
            *c *= -1.0;
        }
    }

    let rule_length = FINITE_DIFF_RULE_LENGTH;
    let size = parameters.len();
    let skip = FINITE_DIFF_RULE_LENGTH / 2;
    let mut a = DMatrix::zeros(size, size);
    for i in 0..size {
        // Coefficient sources are two `&[f64; 7]` arrays and one owned
        // `Vec<f64>` (`backward_coeffs`, built once above since upstream
        // reverses -- and conditionally negates -- `forward_coeffs`).
        // `as &[f64]` unifies all three to one slice type for this match.
        let (start_ind, coeffs): (usize, &[f64]) = if i < skip {
            (i, forward_coeffs as &[f64])
        } else if i < size - skip {
            (i - skip, central_coeffs as &[f64])
        } else {
            (i + 1 - rule_length, &backward_coeffs)
        };
        for (k, &c) in coeffs.iter().enumerate() {
            a[(i, start_ind + k)] = c;
        }
    }

    (a * parameters) / dt.powi(2)
}

/// `toVector`: one [`DVector`] per row of `m`.
pub fn to_vector(m: &DMatrix<f64>) -> Vec<DVector<f64>> {
    (0..m.nrows())
        .map(|row| DVector::from_iterator(m.ncols(), m.row(row).iter().copied()))
        .collect()
}

/// `toString(const Eigen::MatrixXd&)`: one bracketed, comma-separated row
/// per line. Eigen's `IOFormat(4, 0, ", ", "\n", "[", "]")` sets 4
/// *significant* digits under the default (`std::defaultfloat`) stream
/// format, not 4 fixed decimal places; this port uses `{:.4}` (4 decimal
/// places) instead. `toString`/`toVector` have no consumer anywhere in the
/// 1,551-line upstream this crate ports (grepped across `src/`/`include/`) --
/// pure debug-print helpers -- so byte-identical formatting was not worth
/// reproducing Eigen's exact `iostream` precision semantics for; the
/// row/bracket/separator structure upstream's format actually encodes is
/// preserved.
pub fn matrix_to_string(m: &DMatrix<f64>) -> String {
    let mut rows = Vec::with_capacity(m.nrows());
    for row in 0..m.nrows() {
        let mut cols = Vec::with_capacity(m.ncols());
        for col in 0..m.ncols() {
            cols.push(format!("{:.4}", m[(row, col)]));
        }
        rows.push(format!("[{}]", cols.join(", ")));
    }
    rows.join("\n")
}

/// `toString(const Eigen::VectorXd&)`: `data.transpose()` formatted as a
/// single bracketed row. See [`matrix_to_string`] for the precision caveat.
pub fn vector_to_string(v: &DVector<f64>) -> String {
    let cols: Vec<String> = v.iter().map(|x| format!("{x:.4}")).collect();
    format!("[{}]", cols.join(", "))
}

/// `toString(const std::vector<Eigen::VectorXd>&)`: stacks `rows` into a
/// matrix (one row per element) and formats it as [`matrix_to_string`]
/// would. See [`matrix_to_string`] for the precision caveat.
///
/// Upstream calls `data.front()` unconditionally to size the matrix, which
/// is undefined behavior in C++ when `data` is empty. This port panics
/// instead, with a message naming the cause, rather than reproducing that
/// undefined behavior.
pub fn rows_to_string(rows: &[DVector<f64>]) -> String {
    assert!(
        !rows.is_empty(),
        "rows_to_string: upstream's toString(vector<VectorXd>) calls data.front() unconditionally, which is undefined behavior on an empty vector in C++ -- this port fails fast here instead"
    );
    let ncols = rows[0].len();
    let mut m = DMatrix::zeros(rows.len(), ncols);
    for (i, row) in rows.iter().enumerate() {
        m.row_mut(i).copy_from(&row.transpose());
    }
    matrix_to_string(&m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn finite_difference_matrix_for_position_order_is_scaled_identity() {
        // FINITE_CENTRAL_DIFF_COEFFS[Position] = [0,0,0,1,0,0,0]: the only
        // nonzero coefficient lands on the diagonal for every row, at every
        // depth (forward/central/backward branch alike), so the whole
        // matrix reduces to `multiplier * I` -- hand-verifiable without a
        // reference implementation.
        let m = generate_finite_difference_matrix(5, DerivativeOrder::Position, 2.0);
        for i in 0..5 {
            for j in 0..5 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_relative_eq!(m[(i, j)], expected, epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn finite_difference_matrix_truncates_the_stencil_at_a_boundary_row_instead_of_folding_it() {
        // Row 0 of a velocity-order matrix: only j in [0, 3] of the central
        // stencil [-3..3] stay in range (index = 0 + j >= 0), so only
        // FINITE_CENTRAL_DIFF_COEFFS[Velocity][3..7] are written, at columns
        // 0..4 -- coefficients for j in [-3, -1] are dropped, not clamped
        // onto column 0. This is the exact behavior the module doc's
        // "dead clamp" note describes; if this port's port ever re-adds
        // upstream's dead `index = 0` reassignment as live code, this test
        // catches the resulting row-0 change.
        let m = generate_finite_difference_matrix(5, DerivativeOrder::Velocity, 1.0);
        let velocity = FINITE_CENTRAL_DIFF_COEFFS[DerivativeOrder::Velocity as usize];
        for col in 0..4 {
            assert_relative_eq!(m[(0, col)], velocity[col + 3], epsilon = 1e-12);
        }
        assert_relative_eq!(m[(0, 4)], 0.0, epsilon = 1e-12);
    }

    #[test]
    fn finite_difference_matrix_of_a_single_timestep_has_only_the_center_coefficient() {
        // num_time_steps = 1: the entire stencil except j = 0 falls outside
        // [0, 1), so only the center coefficient (index 3 of 7) survives.
        let m = generate_finite_difference_matrix(1, DerivativeOrder::Acceleration, 1.0);
        let acceleration = FINITE_CENTRAL_DIFF_COEFFS[DerivativeOrder::Acceleration as usize];
        assert_relative_eq!(m[(0, 0)], acceleration[3], epsilon = 1e-12);
    }

    #[test]
    fn differentiate_position_order_is_the_identity_stencil_divided_by_dt_squared() {
        // Position order's central/forward/backward coefficient rows are
        // each a single 1.0 at the position that lands back on the row's
        // own index (worked through by hand in this function's own doc
        // comment reasoning) -- so A = I regardless of which of the three
        // branches fills a given row, and `differentiate` reduces to plain
        // elementwise division by dt^2.
        //
        // 9 elements, not 7 (== FINITE_DIFF_RULE_LENGTH): with skip = 3 and
        // rule_length = 7, the backward branch's `start_ind = i + 1 -
        // rule_length` needs `i >= rule_length - 1 = 6`, and that branch
        // starts at `i = size - skip`; that requires `size - skip >= 6`, ie
        // `size >= 9`. At size 7 the backward branch's first two rows (i =
        // 4, 5) would need a negative `start_ind` -- upstream's C++
        // undefined behavior, this port's bounds-checked panic -- so 7 was
        // not actually a valid size to exercise all three branches with;
        // 9 is the smallest one that is. This also exercises all three
        // branches: forward i=0..2, central i=3..5, backward i=6..8.
        let parameters = DVector::from_vec((1..=9).map(f64::from).collect());
        let derivatives = differentiate(&parameters, DerivativeOrder::Position, 2.0);
        for i in 0..9 {
            assert_relative_eq!(derivatives[i], parameters[i] / 4.0, epsilon = 1e-12);
        }
    }

    #[test]
    #[should_panic]
    fn differentiate_panics_instead_of_corrupting_memory_when_parameters_is_too_short_for_the_stencil()
     {
        // size = 2 < FINITE_DIFF_RULE_LENGTH: upstream's C++ segment write
        // here is undefined behavior; this port panics instead (see this
        // function's doc comment).
        let parameters = DVector::from_vec(vec![1.0, 2.0]);
        let _ = differentiate(&parameters, DerivativeOrder::Position, 1.0);
    }

    #[test]
    fn smoothing_matrix_diagonal_is_exactly_one_over_num_timesteps_by_construction() {
        // The final loop scales column t by 1/(num_timesteps * M(t,t)), so
        // the new diagonal entry (t,t) is M(t,t) * 1/(num_timesteps *
        // M(t,t)) = 1/num_timesteps for every t and every invertible R --
        // an invariant of the algorithm's own structure, independent of
        // dt or R's actual numeric values, so it needs no reference
        // implementation to check against.
        for num_timesteps in [1, 2, 5, 20] {
            let m = generate_smoothing_matrix(num_timesteps, 0.1).unwrap_or_else(|| {
                panic!(
                    "num_timesteps={num_timesteps}: control_cost_matrix_R was unexpectedly singular"
                )
            });
            for t in 0..num_timesteps {
                assert_relative_eq!(m[(t, t)], 1.0 / num_timesteps as f64, epsilon = 1e-9);
            }
        }
    }

    #[test]
    fn smoothing_matrix_is_square_with_the_requested_number_of_timesteps() {
        let m = generate_smoothing_matrix(8, 0.05).unwrap();
        assert_eq!(m.nrows(), 8);
        assert_eq!(m.ncols(), 8);
    }

    #[test]
    fn smoothing_matrix_for_zero_timesteps_is_the_empty_matrix_not_a_panic() {
        // nalgebra's FullPivLU::is_invertible computes `nrows() - 1`
        // unconditionally, which underflows a usize for a 0x0 matrix; this
        // boundary is handled before that call is reached (see this
        // function's own "num_timesteps == 0" doc section).
        let m = generate_smoothing_matrix(0, 0.1).expect("0x0 is trivially invertible");
        assert_eq!(m.nrows(), 0);
        assert_eq!(m.ncols(), 0);
    }

    #[test]
    fn to_vector_then_rows_to_string_round_trips_through_matrix_to_string() {
        let m = DMatrix::from_row_slice(2, 3, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let rows = to_vector(&m);
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0], DVector::from_vec(vec![1.0, 2.0, 3.0]));
        assert_eq!(rows[1], DVector::from_vec(vec![4.0, 5.0, 6.0]));
        assert_eq!(rows_to_string(&rows), matrix_to_string(&m));
    }

    #[test]
    fn vector_to_string_is_one_bracketed_comma_separated_row() {
        let v = DVector::from_vec(vec![1.0, -2.5, 3.0]);
        assert_eq!(vector_to_string(&v), "[1.0000, -2.5000, 3.0000]");
    }

    #[test]
    #[should_panic(expected = "upstream's toString(vector<VectorXd>) calls data.front()")]
    fn rows_to_string_panics_instead_of_replicating_ub_on_empty_input() {
        // A bare `#[should_panic]` cannot tell this named guard's panic
        // apart from the raw index-out-of-bounds panic `rows[0].len()`
        // would raise on its own if the guard were deleted (bite-checked:
        // neutralizing the guard left this test green, panicking on the
        // indexing instead). The `expected` substring pins the guard's own
        // message.
        let rows: Vec<DVector<f64>> = Vec::new();
        let _ = rows_to_string(&rows);
    }
}
