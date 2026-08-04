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
sweeps `num_timesteps` 1..=200 (dense 1..=60, checkpoints to 200 --
an order of magnitude past the largest `num_timesteps` any test or
fixture in this workspace uses, `solve_with_60_timesteps_converges`'s
60) and confirms `MultivariateGaussian::new` never returns `None` in
that range.

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
