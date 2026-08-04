# Claim audit: moveit-planners-stomp

Per PORTING-PLAN.md §175. One row per item, appended as found, not
batched at report time. `evidence` is the upstream `file:line` actually
opened this round -- inference from the port is not evidence.

Upstream root for this crate: `/home/stevek/work/moveit2` (pinned
`e017c91ee12984393a28ba246075c65f69cde3bf`), `moveit_planners/stomp/`.

## §172 sweep (round 33), upstream-first per `0ad8c67`

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-stomp/src/cost_functions.rs:192-196` (`cost_function_from_state_validator`'s Gaussian-smoothing kernel bounds) | Port reproduces upstream's own `double`->`long` narrowing (`static_cast<long>(sigma)` then implicit narrowing of `mu - that*4` on assignment) via matching `as i64` casts; `sigma`/`mu` are derived only from bounded window indices (`start`/`end` <= `num_timesteps`), always finite/non-negative/far under `i64::MAX` in every reachable call, so no boundary exists where Rust's `as` and C++'s UB actually diverge | CONFIRMED distinct (parity-matching, no reachable divergence) | `/home/stevek/work/moveit2/moveit_planners/stomp/include/stomp_moveit/cost_functions.hpp:167-171` -- `const long kernel_start = mu - static_cast<long>(sigma) * 4; const long kernel_end = mu + static_cast<long>(sigma) * 4; const long bounded_kernel_start = std::max(0l, kernel_start); const long bounded_kernel_end = std::min(static_cast<long>(values.cols()) - 1, kernel_end);` | (none; pre-existing doc at `cost_functions.rs:65-74` already covers this) |
| `moveit2 moveit_planners/stomp/include/stomp_moveit/{stomp_moveit_task,conversion_functions,filter_functions,noise_generators}.hpp`, `moveit_planners/stomp/src/stomp_moveit_planning_context.cpp`, `moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp` (not this crate's, see moveit-sampling) | No other float-derived `int`/`unsigned`/`size_t`/`long` narrowing in the files this crate ports | CONFIRMED, 0 additional hits | Full-file grep of each, read in this tree; the one other `size_t` hit (`noise_generators.hpp:66`, `m.rows() - std::abs(diag_index)`) is integer-derived | (none) |
| `crates/moveit-planners-stomp/src/*.rs` (port-side anchor: `as i8..u128/usize` receiving `f64`) | The only hits are the 5 lines inside the one kernel-bounds block above; no other file in this crate has the anchor at all | CONFIRMED, 5 sites, all covered by the row above | Read in this tree only | (none) |

## §167.5 license-citation finding (round 33), separate from §172

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-stomp/src/lib.rs:5-6` (pre-fix) | Citation `moveit_planners/stomp/` (bare directory) resolves to 20 upstream files but the license gate's `len(resolved) == 1` branch skips retention checking for it entirely -- not a compliance miss (checked: the only file among those 20 with a different (year, holder) pair, `multivariate_gaussian.{h,hpp}` 2009 Willow Garage, is not ported by this crate and is already retained by moveit-sampling), but an unchecked citation | CONFIRMED, narrowed | `/home/stevek/work/moveit2/moveit_planners/stomp/` directory listing (20 files) cross-referenced against this crate's own per-module citations, which name exactly 6 of them | `0ca9158` |

## §167.6 re-check (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-stomp/src/lib.rs:5-11` (post-`0ca9158`) | Still the directory+indented-filenames shape (six files named under two directory lines), not a bare directory with nothing indented beneath it -- the newly-closed parser hole does not apply here | CONFIRMED, no regression | Read in this tree; `tools/ci/verify-upstream-license-provenance.sh` run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |

## §172 row 1 (round 33) backed with a measured boundary test, per PORTING-PLAN.md §172

The round-33 row above classified `kernel_start`/`kernel_end` as "no
reachable divergence" without a test pinning what "reachable" actually
bounds. That gap is closed here: `kernel_start`/`kernel_end`'s bound
computation was extracted to a private `kernel_bounds(mu, sigma,
num_timesteps) -> (usize, usize)` (`cost_functions.rs`, no behavior
change -- same six lines, same order, same casts) so it can be called
directly with `start`/`end`/`num_timesteps` extremes without needing to
allocate a `DMatrix` large enough to reach them.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-stomp/src/cost_functions.rs` `kernel_bounds` (`cost_functions::tests::kernel_bounds_truncates_sigma_before_multiplying_by_four`) | The truncate-`sigma`-then-multiply-by-4 order actually matters (not just present in the source): for `sigma = 1.5`, truncate-first gives `sigma_offset = 4`, multiply-first (`(sigma * 4.0) as i64`) gives `6` -- a different, observable `kernel_bounds` result (`(2, 10)` vs a wider window). A rewrite to pure-`f64` arithmetic would silently change the returned bounds and fail this test | CONFIRMED, cast order pinned by a differential assertion inside the test itself | Computed directly in the test, both formulas evaluated and asserted unequal before asserting the production result | `6d1b9dc` |
| `crates/moveit-planners-stomp/src/cost_functions.rs` `kernel_bounds` (`cost_functions::tests::kernel_bounds_at_the_dmatrix_allocation_ceiling_does_not_overflow`) | The round-33 "no reachable divergence" claim rests on "`sigma`/`mu` ... always finite/non-negative/far under `i64::MAX`" without stating what bounds `sigma`. The actual reachable ceiling is not `usize::MAX`/`i64::MAX`: `num_timesteps = values.ncols()`, and a `DMatrix<f64>`'s backing `Vec<f64>` cannot allocate past `isize::MAX` bytes, so `num_timesteps <= isize::MAX / size_of::<f64>() = 1_152_921_504_606_846_975` for any real (single-row, maximally generous) trajectory. `sigma_offset` (`(sigma as i64) * 4`) only overflows `i64` once `sigma > i64::MAX / 4 ~= 2.305e18`, which requires `num_timesteps > ~4.611e18` -- about 4x past the allocation ceiling. Measured directly (not asserted in prose): calling `kernel_bounds` with `start = 0`, `end = max_reachable_num_timesteps - 1` under nextest's default dev profile (`overflow-checks = true`) does not panic, and a `checked_mul(4)` computed alongside it returns `Some` | CONFIRMED, no reachable divergence -- measured against the actual `DMatrix` allocation ceiling, not asserted | `crates/moveit-planners-stomp/src/cost_functions.rs` `kernel_bounds`; `std::mem::size_of::<f64>()`/`isize::MAX` are the same limits `Vec`'s allocator enforces (`alloc::raw_vec`, capacity check against `isize::MAX` bytes) | `6d1b9dc` |

### Expiry (§153.1)

The 4x margin is a property of `DMatrix<f64>`'s `Vec`-backed storage,
not of `kernel_bounds` itself, which takes a plain `usize` and has no
cap of its own. This claim expires -- becomes false, not merely
unverified -- the moment any of the following becomes true for
`cost_function_from_state_validator`'s `values` argument or whatever
supplies `num_timesteps` to it:

- `values` (or its replacement) is no longer backed by a single
  contiguous `Vec<f64>` capped at `isize::MAX` bytes -- e.g. a
  memory-mapped, chunked, or lazily-streamed trajectory
  representation whose column count is not bounded by that allocator
  check.
- The index type carrying `num_timesteps`/`start`/`end` widens past 64
  bits (a 128-bit index, or an arbitrary-precision count).
- `kernel_bounds`'s multiplier (currently `* 4`, hardcoding `+/-
  4*sigma`) changes to a value that moves the ~4.611e18 overflow
  threshold below the ~1.153e18 allocation ceiling.

If any of these lands, `kernel_bounds`'s two boundary tests
(`kernel_bounds_at_the_dmatrix_allocation_ceiling_does_not_overflow`,
and the ceiling arithmetic in its doc comment) must be re-derived
against the new ceiling before this row can be re-asserted CONFIRMED --
do not carry it forward unchanged.

## §194 port-only API sweep (this round): `moveit-planners-stomp`

This crate's share of the cross-crate sweep triggered by `a682f63`
(full rationale and `moveit-stomp-core`'s own rows in that crate's
claim-audit doc). Anchor and method identical: every `pub fn`/`pub
struct` in `crates/moveit-planners-stomp/src/*.rs`, cross-referenced
against `moveit2 moveit_planners/stomp/` for an upstream counterpart.

| API | port-only? | upstream state newly reachable | invariant at risk? |
|---|---|---|---|
| `ComposableTask::new(...)` | no -- direct port of upstream `stomp_moveit::ComposableTask`'s constructor (see `composable_task.rs`'s own module doc) | none | no |
| `UnparameterizedTrajectory` (public `way_point_count`/`into_uniformly_timed`, private wrapped `RobotTrajectory`) | yes -- no upstream type; invented this port's own "Deviation: unparameterized-by-construction" (round 21) | none -- the opposite direction: this type exists specifically to make an upstream footgun (reading the placeholder `dt = 0.1` as if it were real timing) *unreachable*, not to open new state | no -- protective, not permissive |
| `PlanRequest` (plain public-field struct, no methods) | yes -- a Rust named-argument idiom for `plan`'s parameter list, no upstream counterpart | none -- every field is a reference of a type (`&RobotState`, `&JointModelGroup`, `Option<&RobotTrajectory>`) already public to any caller holding it; bundling grants no new capability | no |
| `plan(config, cost_fn, request, rng, cancel_handle)` | yes (round 24) -- upstream's equivalent logic is inline inside `StompPlanningContext::solve`, itself unported (D1/D2) | `cancel_handle: CancelHandle` is taken and forwarded unmodified into `Stomp::with_cancel_handle` (`planner.rs:466`) -- this *is* the surface `a682f63` already fixed at its root; `plan` adds no cancellation entry point of its own beyond that one | no -- covered by `moveit-stomp-core`'s fix, not a second instance |

**Conclusion:** no port-only API in this crate opens new
upstream-invariant-protected state on its own; `plan`'s
`cancel_handle` parameter is the same already-fixed surface as
`moveit-stomp-core::Stomp::with_cancel_handle`, not an independent
finding.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-stomp/src/*.rs`, every `pub fn`/`pub struct` | No port-only API in this crate carries independent invariant risk; `plan`'s cancellation surface is `with_cancel_handle`'s, already fixed | CONFIRMED, 4 APIs enumerated and classified above | Read every public item in this tree, cross-referenced against `/home/stevek/work/moveit2/moveit_planners/stomp/` | (pending, see report) |

## `MultivariateGaussian::new`'s fallibility vs. D14/§199 (this round)

Mirror image of §194: `moveit-sampling::MultivariateGaussian::new`
(not this crate, but this crate is its one production caller) is
fallible where upstream's constructor is not -- it returns `None` for
a non-positive-definite `covariance` that upstream would have
"accepted" (silently producing `NaN`-poisoned sampling state; see that
crate's own module doc, "Deviation: construction can fail"). D14
(`moveit-constraints`, this round, §199): a wire default upstream
assigns meaning to must not be rejected. Same shape of question here:
does this port's stricter constructor reject an input a real caller
(a message, a config file) can actually reach?

**Call-site inventory.** `rg 'MultivariateGaussian::new'` across the
workspace: every hit outside `moveit-sampling`'s own tests and doc
comments is `crates/moveit-planners-stomp/src/noise_generators.rs:106`,
inside `normal_distribution_generator`. `stomp.rs:1162`'s hit is
`moveit-stomp-core`'s own `#[cfg(test)]` module, not production. One
production call site in the entire workspace.

**Does `stddev` (the caller/config-facing parameter) reach it?** No --
structurally, not just "not observed to." `normal_distribution_generator`'s
`covariance` argument to `MultivariateGaussian::new` (`noise_generators.rs:106`)
is `full_piv_lu_try_inverse_or_empty(acceleration^T * acceleration)`,
normalized by its own max-abs entry -- a function of `num_timesteps`
alone. `stddev` is applied only afterward, as a per-row scale on
already-sampled noise (`noise_generators.rs:124-126`), never touching
`covariance`. `normal_distribution_generator`'s only production caller,
`plan` (`planner.rs:452-453`), always builds `stddev` as
`vec![DEFAULT_NOISE_STDDEV; config.num_dimensions]` -- a hardcoded
constant, not read from a message field per-dimension -- but that is
moot given `stddev` cannot reach `covariance` regardless of what a
caller supplies.

**Does `num_timesteps` (the one input that does reach it) reach it?**
Checked, not assumed: `acceleration^T * acceleration`'s invertibility
is already gated by this function's first `ok_or_else`
(`noise_generators.rs:96-101`); once that passes, `covariance` is a
positive-scalar-normalized inverse of an invertible Gram matrix, which
is mathematically positive-definite by construction (a Gram matrix's
inverse is PD whenever the Gram matrix is invertible). This is the
identical premise `filter_functions::simple_smoothing_matrix`'s own
test-module note already documents for the same `A^T * A` shape
(`filter_functions.rs:142-152`, "no realistic `(num_timesteps, dt)`
input... makes it singular"). Backed empirically here too, not just
asserted: `noise_generators::tests::num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects`
checks `num_timesteps` `1..=200` **contiguously**, no gap. Not always
true: this round found and closed two problems in how this coverage
was previously described and justified. First (§189, corrected in
`8351f8d` and again here): the coverage used to be `1..=60`
contiguous plus four sampled points (`80`, `100`, `150`, `200`), and
both this doc and the test's own doc comment described it more
broadly than that. Second (found by this round's own §189 sweep): the
stated reason for sampling instead of covering contiguously --
"past the largest `num_timesteps` any test or fixture in this
workspace uses, `solve_with_60_timesteps_converges`'s `60`" -- was
itself an over-broad claim, false on both counts
(`cost_functions.rs:432` uses `num_timesteps = 100` in this crate's
own tests; `solve_with_60_timesteps_converges` doesn't even exercise
this function's covariance path, its `MultivariateGaussian::new` call
is a different, diagonal-and-trivially-PD covariance in
`moveit-stomp-core`'s own `DummyTask::new`, not the
acceleration-Gram-matrix-inverse shape `normal_distribution_generator`
builds). The actual reason coverage was non-contiguous was cost, not
usage: a full contiguous `1..=200` sweep's `O(n^3)`
`full_piv_lu`/Cholesky cost measured past 100s under the workspace's
then-`opt-level = 0` dev profile. `e733f19` raised the dev profile to
`opt-level = 1`; under that profile the full contiguous sweep measures
`0.7s` -- see Item 3 below for the conditioning numbers that motivated
re-measuring the cost, and the closed gap that followed from it.

**Conclusion:** not the same defect family as D14. There is no
upstream-accepted wire value this port's stricter `new` silently
drops -- `stddev` cannot reach the rejection path at all, and
`num_timesteps` cannot produce a `covariance` upstream would treat
differently (upstream's own `NaN`-producing path is for a genuinely
indefinite covariance, and this port's derived covariance cannot be
indefinite once the first invertibility check passes). Closed, not
carried forward as an open risk.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-stomp/src/noise_generators.rs:106` (`MultivariateGaussian::new`'s one production call site) | `stddev` cannot reach `covariance` (structurally disconnected); `num_timesteps` cannot produce a `covariance` `MultivariateGaussian::new` rejects, for any value this workspace uses | CONFIRMED closed, not D14's shape | `rg 'MultivariateGaussian::new'` workspace-wide (1 production site); `noise_generators::tests::num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects`, `cargo nextest run -p moveit-planners-stomp` | (pending, see report) |

## Item 3: floating-point conditioning behind the Gram-matrix-PD argument (this round)

The paragraph above rests on "a Gram matrix's inverse is
positive-definite whenever the Gram matrix is invertible" -- true in
exact arithmetic, not automatically true once nalgebra's
`Cholesky::new` (the function `MultivariateGaussian::new` actually
calls to reject) runs in `f64`. Read directly from
`nalgebra-0.35.0/src/linalg/cholesky.rs:190-265` this round: `new`
returns `None` when an in-place Cholesky-Crout pivot's Schur
complement is exactly zero or its square root fails -- a
floating-point rounding failure mode that depends on conditioning, not
a literal "any eigenvalue <= 0" check. So the PD argument's actual
question is empirical: how far is the measured conditioning from that
rounding-failure zone at the `num_timesteps` this crate checks?

**Measured**, via a new permanent test,
`noise_generators::tests::acceleration_gram_matrix_conditioning_has_wide_margin_from_cholesky_failure`
(`generate_finite_difference_matrix(n, DerivativeOrder::Acceleration,
1.0)`, `raw_covariance = acceleration^T * acceleration`,
`raw_covariance.symmetric_eigenvalues()`):

- `n = 60`: `min_eig ~= 7.107e-6`, `max_eig ~= 28.397`, condition
  number `~= 3.996e6`.
- `n = 200`: `min_eig ~= 5.986e-8`, `max_eig ~= 28.440`, condition
  number `~= 4.751e8`.

**Distance from the failure zone.** A pivot risks rounding to
non-positive once the condition number approaches roughly `1 / (n *
f64::EPSILON)`. At `n = 200` that threshold works out to roughly
`2.25e13` -- the measured `4.751e8` is about four to five orders of
magnitude below it, not close. The measured growth from `n = 60` to
`n = 200` is consistent with the known `O(n^4)` conditioning-growth
law for a second-derivative finite-difference Gram matrix; extrapolating
that law to find where conditioning would reach the failure threshold
gives `n` around `2950` -- about 15x past the largest sampled point
(`200`) and about 49x past the largest `num_timesteps` any real
fixture in this workspace uses (`60`).

**Conclusion for the coordinator's original conditional (prior
round):** it did not trigger -- the conditioning at `n = 60` and
`n = 200` is not close to Cholesky's practical failure zone, so the
non-contiguous four-point coverage above `n = 60` was not, on that
question alone, inadequate.

**But that measurement also closes a question it left open (this
round):** the earlier round's reason for leaving coverage
non-contiguous was `O(n^3)` cost under the workspace's then-current
dev profile, not the conditioning question above -- and cost is a
number that can go stale independently of conditioning. `e733f19`
(this round, unrelated to this test) raised the workspace dev profile
from `opt-level = 0` to `opt-level = 1`; re-measured under the new
profile, the full `1..=200` contiguous sweep costs `0.7s`, the single
slowest test in this crate but not by a margin that matters. The
stale-cost reason for sampling no longer holds, so
`num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects`
now covers `1..=200` contiguously with no gap, closing the question
rather than re-justifying the old gap with the (still true, but no
longer load-bearing) conditioning margin.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-stomp/src/noise_generators.rs` (Gram-matrix-PD argument, floating-point conditioning) | Condition number of `acceleration^T * acceleration` is `~3.996e6` at `n=60`, `~4.751e8` at `n=200` -- four to five orders of magnitude below the estimated Cholesky rounding-failure threshold; conditioning alone did not require contiguous coverage, but re-measuring the sampling reason (cost) under the new dev profile did -- coverage is now contiguous `1..=200`, not sampled | CONFIRMED, measured not assumed | `noise_generators::tests::acceleration_gram_matrix_conditioning_has_wide_margin_from_cholesky_failure`, `noise_generators::tests::num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects` (now `1..=200` contiguous, `0.7s`), `cargo nextest run -p moveit-planners-stomp`; `nalgebra-0.35.0/src/linalg/cholesky.rs:190-265` read for the actual failure condition | (pending, see report) |

### Bite-check: does the pinned test add signal, checked by mutation (this round)

A pinned measurement is only a real regression guard if some concrete
mutation reddens it independently. Two mutations run against this
worktree, gated, then reverted (`git diff` confirmed clean before
committing):

1. **`FINITE_CENTRAL_DIFF_COEFFS`'s acceleration row**
   (`crates/moveit-stomp-core/src/utils.rs`), `-30.0 / 12.0` to
   `-30.5 / 12.0`. `cargo nextest run -p moveit-stomp-core -p
   moveit-planners-stomp`: 2 of 78 tests reddened --
   `acceleration_gram_matrix_conditioning_has_wide_margin_from_cholesky_failure`
   itself (its pinned `expected_cond` assertion), and, as unrelated
   collateral (the same coefficient table feeds `Stomp::solve`'s own
   differentiation, not anything under test here),
   `moveit-stomp-core::stomp::tests::each_convergence_test_fails_if_the_accept_path_update_is_disabled`.
   No test that checks `normal_distribution_generator`'s actual
   noise/covariance *output* caught this mutation -- confirming this
   pinned test adds signal a coefficient-table regression would
   otherwise slip past.
2. **`normal_distribution_generator`'s own call**
   (`noise_generators.rs:94`), `DerivativeOrder::Acceleration` to
   `DerivativeOrder::Velocity` -- a wrong-derivative-order bug in the
   function this test's own doc comment used to claim protecting.
   Same command: 6 of 78 tests reddened --
   `noise_generators::tests::{noisy_values_equal_values_plus_noise,
   repeated_calls_draw_fresh_noise_via_advancing_rng_state,
   num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects,
   stddev_scales_the_noise_magnitude}`,
   `planner::tests::{plan_finds_a_lower_cost_trajectory_than_the_initial_straight_line_through_an_obstacle,
   plan_overrides_num_timesteps_from_a_nonempty_seed_trajectory}` --
   and the pinned conditioning test was **not** among them; it stayed
   green. It cannot catch this class of bug: it recomputes
   `acceleration^T * acceleration` directly from
   `generate_finite_difference_matrix`, the same two lines this
   function's own body runs, rather than calling
   `normal_distribution_generator` itself, so a bug in how *this
   function* uses that matrix is invisible to it.

**Conclusion:** the pinned test is a real, non-redundant regression
guard against one specific class of mutation (a coefficient-magnitude
change to the shared `generate_finite_difference_matrix`), where it is
the *only* test in the workspace that reddens. It is not a guard
against this function's own construction bugs, and its own doc comment
previously (incorrectly, found by this same bite-check) claimed the
latter too -- corrected in `noise_generators.rs` to state precisely
which mutation class it does and does not catch, backed by the
mutation classes above rather than by both being merely asserted.

### Sweep: "test recomputes what the subject also computes", both stomp crates (this round)

The bite-check above generalizes: a test whose assertion is computed
by the test rather than read from the subject's actual call is blind
to a bug in how the subject constructs that value. Every test in both
crates was read for this shape -- **81 of 81** (`cargo nextest run -p
moveit-stomp-core -p moveit-planners-stomp` reports 81), not a
grep-filtered subset, since the anchor ("recomputes a quantity the
subject also computes, without calling the subject") has no single
literal pattern to `rg` for. `rg` was used to enumerate every call
site of the shared helpers a duplicate-computation test would have to
route through (`generate_finite_difference_matrix`,
`generate_smoothing_matrix`, `symmetric_eigenvalues`) as a starting
set, then every hit and every remaining test not on that list was read
by hand.

**Anchor:** a test body that reconstructs a quantity the subject also
computes, without ever calling the subject to get it.

**Anchor as a concrete `rg` pattern (round: margin audit).** The shape
itself ("reconstructs a quantity without calling the subject") has no
single literal regex -- whether a hit is the anchor or "distinct"
requires reading whether the enclosing test also calls the real
subject, which is exactly what the table below already does per row.
What *is* mechanizable is the **first-pass filter**: every call site
of a shared helper capable of producing the kind of derived numeric
quantity a duplicate-computation test would reconstruct. Re-run this
command and diff its output against the table below -- any new line
not already classified in a row is a new candidate to read by hand and
add a row for; this replaces re-reading all 81 (now 84, after this
round's test-splitting commit) tests from scratch:

```
rg -n --type rust \
  -e 'generate_finite_difference_matrix\(' \
  -e 'generate_smoothing_matrix\(' \
  -e 'symmetric_eigenvalues\(' \
  crates/moveit-stomp-core/src crates/moveit-planners-stomp/src
```

Run as of this round this returns 19 lines. 9 are each helper's own
`pub fn`/`fn` definition line, its internal production-code call sites
(e.g. `generate_smoothing_matrix` calling
`generate_finite_difference_matrix` inside `utils.rs`, `Stomp::new`
calling it in `stomp.rs`), and doc-comment mentions -- none of those
are candidates, they are the subject's own implementation, not a test
reconstructing it. The remaining 10 are inside a `#[cfg(test)] mod
tests { ... }` block and map onto 7 of the 10 rows in the table below
(`utils.rs`'s three finite-difference tests, its three smoothing-matrix
tests, `stomp.rs::DummyTask::new`, `filter_functions.rs`'s
`simple_smoothing_matrix_applies_...`, and
`noise_generators.rs`'s `acceleration_gram_matrix_conditioning_...`).
The other 3 rows (`stomp.rs::interpolate`, `planner.rs::linear_interpolation`,
`planner.rs::plan_finds_a_lower_cost_trajectory_...`'s inline duplicate)
are a different sub-shape -- a parallel reimplementation of
interpolation, not reuse of one of these three helpers -- and this
pattern cannot find them; they were only found by the original hand
read and have no mechanized filter here. A future re-run should
discard any hit outside a `#[cfg(test)]` module before diffing the
remaining lines against the table, then still hand-read for the
interpolation-duplication sub-shape separately.

This is a maintained allowlist, not a closed-form detector: it only
finds tests routing through helpers already on the `-e` list. Extend
the list whenever a new shared numerical helper crosses either crate's
internal module boundary (a new `pub(crate)` or private function in
`utils.rs`/`filter_functions.rs`/`noise_generators.rs` that computes a
derived matrix, cost, or statistic another module might independently
recompute in a test). A hit this filter cannot catch -- an ad hoc
formula duplicated inline with no shared helper behind it at all --
still requires the hand-read this sweep already did once; the filter
narrows the set that needs re-reading after this baseline, it does not
eliminate hand-reading permanently.

**Sites** (`generate_finite_difference_matrix`/`generate_smoothing_matrix`/`symmetric_eigenvalues`
call sites inside `#[cfg(test)]` modules, plus the two already-documented
duplicate-formula test helpers):

| site | classification | why |
|---|---|---|
| `utils.rs::finite_difference_matrix_for_position_order_is_scaled_identity` (`generate_finite_difference_matrix` at line 523) | distinct | calls the actual subject and compares its output against a hand-derived identity matrix -- not a parallel computation of the same formula |
| `utils.rs::finite_difference_matrix_truncates_the_stencil_at_a_boundary_row_instead_of_folding_it` (line 542) | distinct | same -- subject called, output compared against the static `FINITE_CENTRAL_DIFF_COEFFS` table (a raw input constant, not a re-derived output) |
| `utils.rs::finite_difference_matrix_of_a_single_timestep_has_only_the_center_coefficient` (line 554) | distinct | same reasoning as the previous row |
| `utils.rs::smoothing_matrix_diagonal_is_exactly_one_over_num_timesteps_by_construction` / `smoothing_matrix_is_square_...` / `smoothing_matrix_for_zero_timesteps_...` (lines 605/618/629) | distinct | `generate_smoothing_matrix` **is** the subject under test in each of these -- there is no second call anywhere else to be blind to |
| `stomp.rs::DummyTask::new` (line 1154, inside `mod tests`) | distinct | builds the test harness's own smoothing filter (fixture construction), never compared against `Stomp`'s internal state -- not an expected-value computation at all |
| `filter_functions.rs::simple_smoothing_matrix_applies_the_smoothing_matrix_to_each_row` (line 158) | distinct, but closest call -- reasoned through explicitly | the test **does** call the actual subject (`filter(&values, &mut filtered_values)`) and asserts on its real return value; the `expected_m = generate_smoothing_matrix(...)` call only supplies a known-correct matrix for the test's own independently-written `filtered_before * expected_m.transpose()` formula. A bug in `simple_smoothing_matrix`'s own arithmetic (skipping the transpose, swapping multiplication order) changes the subject's *actual* output while the test's independently-written expression stays fixed, so it would be caught -- unlike the anchor case, this is "subject called, output checked", not "subject never called" |
| `noise_generators.rs::acceleration_gram_matrix_conditioning_has_wide_margin_from_cholesky_failure` (line 212) | **same defect, already found and closed in the prior round** | recomputes `acceleration^T * acceleration` directly from `generate_finite_difference_matrix` and never calls `normal_distribution_generator` at all -- exactly the anchor. Already bite-checked (see above): mutating `normal_distribution_generator`'s own call left this test green while 6 others reddened. Already remediated by doc correction, not by changing what the test measures (its actual subject is the Gram matrix's own conditioning, a legitimate thing to pin directly) -- re-confirmed this round as the correct, already-closed classification, not a new site |
| `stomp.rs::interpolate` (`mod tests`, line 1276) | distinct, deliberately so | doc comment on the function itself: kept as "a second, separately-maintained copy" of `compute_linear_interpolation` *on purpose*, specifically so calling the real `compute_linear_interpolation` here would not silently mask a bug in the thing under test when scoring `LinearInterpolation`-initialized cases against it |
| `planner.rs::linear_interpolation` (`mod tests`, line 731) | distinct, deliberately so | same shape and same reasoning, own doc comment: "replicated here because it is not `pub`" -- used to assert the subject's cancelled-early output equals this independent reference bit-for-bit, which is the correct use of a parallel implementation (verifying the subject's real output), not the anchor's failure mode (never reading the subject's output at all) |
| `planner.rs::plan_finds_a_lower_cost_trajectory_than_the_initial_straight_line_through_an_obstacle` (inline duplicate at lines 699-705, unnamed) | distinct | same linear-interpolation formula inlined to build a known oracle *input* (the straight-line trajectory the obstacle sits across), then evaluated through `eval_cost_fn` (the real subject) for both the oracle input and `stomp`'s real output -- it is not comparing against a re-derived *output* of `stomp`/`normal_distribution_generator`, only constructing a known starting point |

**Same defect at:** none newly found this round -- the one instance in
the workspace (`acceleration_gram_matrix_conditioning_has_wide_margin_from_cholesky_failure`)
was already identified and remediated in the prior round's bite-check
above.

**Distinct, skip:** all other sites listed, with the table's own
per-row reasoning; the remaining 68 of 81 tests in both crates (every
test not listed above) were read individually and call their subject,
assert against literal/hand-derived expected values, or assert
structural invariants (shape, call counts, `Option`/`Result`
discriminant) -- none independently reconstruct a quantity their own
subject also computes without calling that subject.

No bite-check fix was needed this round: zero new same-defect sites
were found to fix.

## Phase 8 STOMP-side completeness audit, directory-count first (this round)

A file never ported produces no audit row -- a list built from what the
port already cites cannot find it. Counted the two upstream
directories directly instead:

```
$ find /home/stevek/work/stomp -type f | wc -l
9
$ find /home/stevek/work/moveit2/moveit_planners/stomp -type f | wc -l
26
```

### `/home/stevek/work/stomp` (9 files) -- `moveit-stomp-core`

All 9 accounted for, all with a stated reason, no gap found:

| file | status |
|---|---|
| `include/stomp/stomp.h`, `src/stomp.cpp` | ported (`stomp.rs`, 47 symbols enumerated in its own "Completeness audit") |
| `include/stomp/task.h` | ported (`task.rs`, 10 symbols) |
| `include/stomp/utils.h`, `src/utils.cpp` | ported (`utils.rs`, 14 symbols; `utils.cpp` adds nothing beyond `utils.h`'s own declarations, per `lib.rs`) |
| `test/stomp_3dof.cpp` | ported as an acceptance test, not library code -- `stomp.rs`'s own "No upstream reference test with value assertions, closing the gap with `test/stomp_3dof.cpp`" section, re-verified line-by-line round 26 |
| `test/utest.cpp` | deliberately excluded, reason stated (`stomp.rs:36`, "gtest boilerplate") |
| `examples/simple_optimization_task.h`, `examples/stomp_example.cpp` | deliberately excluded, reason stated (`lib.rs:113-115`, "outside this audit's scope") |

### `/home/stevek/work/moveit2/moveit_planners/stomp` (26 files) -- `moveit-planners-stomp` + `moveit-sampling`

| file | status |
|---|---|
| `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml`, `res/stomp_moveit.yaml`, `stomp_moveit_plugin_description.xml`, `test/CMakeLists.txt` | non-code, N/A |
| `conversion_functions.hpp` | ported (`conversion_functions.rs`) |
| `cost_functions.hpp` | ported (`cost_functions.rs`) |
| `filter_functions.hpp` | ported (`filter_functions.rs`) |
| `noise_generators.hpp` | ported (`noise_generators.rs`) |
| `stomp_moveit_task.hpp` | ported (`composable_task.rs`) |
| `conversion_functions.h`, `cost_functions.h`, `filter_functions.h`, `noise_generators.h`, `stomp_moveit_task.h`, `stomp_moveit_planning_context.h` | deprecation stubs (`create_deprecated_headers.py`-generated, `#pragma message` + one `#include` of the `.hpp` twin, confirmed by reading all 6 this round -- **but not previously stated anywhere in this crate's own docs**, unlike `moveit-sampling`'s explicit stub-check of its own two `.h` twins. Documentation gap, not a functional one: the substance (stub, contributes nothing) is now confirmed, just never written down before this round) |
| `math/multivariate_gaussian.h`, `math/multivariate_gaussian.hpp` | ported, but via `moveit-sampling`, not this crate (already fully audited there) |
| `src/stomp_moveit_planning_context.cpp` | partially ported -- `extract_seed_trajectory`/`sample_goal_state` extracted as free functions (round 25); `StompPlanningContext::solve`'s own inline logic remains D1/D2-unported (existing `MultivariateGaussian::new` row above, "itself unported (D1/D2)") |
| `src/stomp_moveit_planner_plugin.cpp` | deliberately excluded, reason stated (`lib.rs:105-115`, ROS-hosted plugin entry point taking `rclcpp::Node::SharedPtr`) |
| `trajectory_visualization.hpp` | deliberately excluded, reason stated (`lib.rs:116-120`, ROS message/tf2-typed signatures) |
| `stomp_moveit_planning_context.hpp` | deliberately excluded, classified this round -- see below |
| `test/test_cost_functions.cpp` | ported this round, by value -- see below |
| `test/test_noise_generator.cpp` | ported this round, by value -- see below |

**`stomp_moveit_planning_context.hpp`, classified this round (closes
gap 1 of 3 named last round).** Read the full file (76 lines): one
type, `class StompPlanningContext : public planning_interface::PlanningContext`
-- every member is either ROS-typed in its own signature
(`solve(planning_interface::MotionPlanResponse&)`,
`solve(planning_interface::MotionPlanDetailedResponse&)`,
`setPathPublisher`/`getPathPublisher` taking/returning
`std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>`),
or a ROS-typed private field (`stomp_moveit::Params params_` --
`generate_parameter_library`-generated, not in this repository tree;
`std::shared_ptr<rclcpp::Publisher<...>> path_publisher_`). No inline
method bodies, no free-standing constants, no type this header defines
that the `.cpp` does not already declare via the same class -- checked
by also reading the `.cpp`'s four short bodies for `terminate()`,
`clear()`, `setPathPublisher()`, `getPathPublisher()`, not just
`solve()` (already the existing "partially ported" row's subject):
`terminate()` is a two-line wrapper over `stomp_->cancel()`, already
covered by this crate's own `Stomp::cancel`/`CancelHandle` cancellation
surface (`a682f63` and later rounds); `clear()` is an empty body;
`setPathPublisher`/`getPathPublisher` are a bare accessor pair over the
ROS-typed `path_publisher_` field. Nothing left uncovered. Same D1/D2
ROS-hosted-glue exclusion already applied to this file's `.cpp` twin
(`solve()`'s body) and to the sibling plugin file -- now written down
explicitly rather than left as "never cited."

**`test/test_cost_functions.cpp` and `test/test_noise_generator.cpp`,
cross-referenced by value this round (closes gaps 2 and 3 named last
round).** Both files were previously covered only by name-match
(`a_fully_valid_trajectory_has_zero_cost_and_is_valid` "looks like"
`testGetCostFunctionAllValidStates`, etc.) -- never opened, never
diffed against this port's own tests literal-for-literal. Both were
read in full this round and every pinned constant extracted:
`test_cost_functions.cpp`'s `TIMESTEPS=100`, `VARIABLES=6`,
`PENALTY=1.0`, the 18-element `INVALID_TIMESTEPS` set (`{0, 10, 11,
12, 25, 26, 27, 46, 63, 64, 65, 66, 67, 68, 69, 97, 98, 99}`),
`costs.sum() == PENALTY * 18 == 18.0` (exact), and
`costs(invalid_timesteps_vec).sum() >= 0.681 * PENALTY`;
`test_noise_generator.cpp`'s `TIMESTEPS=100`, `VARIABLES=6`,
`STDDEV=[0.2; 6]`, and `EXPECT_NE(noise, NOISE)`/`EXPECT_NE(noisy_values,
NOISY_VALUES)` (noise is actually nonzero, not merely shaped correctly).

None of these literals appeared anywhere in this port's own tests --
every existing test in `cost_functions.rs` and `noise_generators.rs`
uses small, differently-valued synthetic fixtures (5-waypoint
trajectories, `stddev` of `0.1`/`10.0`/`1.0`, seeds unrelated to
upstream's RNG), which exercise the same *invariants* (a fully-valid
trajectory has zero cost, an invalid waypoint is penalized, the first
and last columns are pinned to zero noise) but never reproduce
upstream's exact numbers for the exact same input. Per this round's
brief, each unreproduced literal is closed by porting the case rather
than left as a documented gap:

1. `upstream_test_get_cost_function_all_valid_states` and
   `upstream_test_get_cost_function_invalid_states`
   (`cost_functions.rs`, new this round) port
   `testGetCostFunctionAllValidStates`/`testGetCostFunctionInvalidStates`
   using upstream's exact `TIMESTEPS`/`VARIABLES`/`PENALTY`/
   `INVALID_TIMESTEPS` literals and reproduce every one of upstream's
   assertions, including the `0.681 * PENALTY` two-sigma concentration
   claim and `costs.sum() == 18.0` exactly. Both pass against this
   port's own `cost_function_from_state_validator`.
2. `upstream_test_start_end_unchanged` (`noise_generators.rs`, new
   this round) ports `testStartEndUnchanged` using upstream's exact
   `TIMESTEPS`/`VARIABLES`/`STDDEV` literals, and additionally closes a
   gap this cross-reference found independently of the literal check:
   no existing test in this file asserted generated noise is actually
   nonzero (`stddev_scales_the_noise_magnitude`'s ratio assertion would
   pass trivially under a hypothetical all-zero-noise bug, `0 - 100*0
   == 0`). The new test reproduces upstream's `EXPECT_NE(noise, NOISE)`/
   `EXPECT_NE(noisy_values, NOISY_VALUES)`/`EXPECT_NE(VALUES,
   noisy_values)` explicitly.

Unlike `moveit-stomp-core`'s treatment of `test/stomp_3dof.cpp` (explicit
section, re-verified line-by-line), these two upstream test files were
never read at all before this round -- the apparent coverage claimed
previously was a name-match, not a checked one. It is a checked one
now.
