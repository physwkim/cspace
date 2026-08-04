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
| `crates/moveit-planners-stomp/src/cost_functions.rs` `kernel_bounds` (`cost_functions::tests::kernel_bounds_truncates_sigma_before_multiplying_by_four`) | The truncate-`sigma`-then-multiply-by-4 order actually matters (not just present in the source): for `sigma = 1.5`, truncate-first gives `sigma_offset = 4`, multiply-first (`(sigma * 4.0) as i64`) gives `6` -- a different, observable `kernel_bounds` result (`(2, 10)` vs a wider window). A rewrite to pure-`f64` arithmetic would silently change the returned bounds and fail this test | CONFIRMED, cast order pinned by a differential assertion inside the test itself | Computed directly in the test, both formulas evaluated and asserted unequal before asserting the production result | `<pending, see report>` |
| `crates/moveit-planners-stomp/src/cost_functions.rs` `kernel_bounds` (`cost_functions::tests::kernel_bounds_at_the_dmatrix_allocation_ceiling_does_not_overflow`) | The round-33 "no reachable divergence" claim rests on "`sigma`/`mu` ... always finite/non-negative/far under `i64::MAX`" without stating what bounds `sigma`. The actual reachable ceiling is not `usize::MAX`/`i64::MAX`: `num_timesteps = values.ncols()`, and a `DMatrix<f64>`'s backing `Vec<f64>` cannot allocate past `isize::MAX` bytes, so `num_timesteps <= isize::MAX / size_of::<f64>() = 1_152_921_504_606_846_975` for any real (single-row, maximally generous) trajectory. `sigma_offset` (`(sigma as i64) * 4`) only overflows `i64` once `sigma > i64::MAX / 4 ~= 2.305e18`, which requires `num_timesteps > ~4.611e18` -- about 4x past the allocation ceiling. Measured directly (not asserted in prose): calling `kernel_bounds` with `start = 0`, `end = max_reachable_num_timesteps - 1` under nextest's default dev profile (`overflow-checks = true`) does not panic, and a `checked_mul(4)` computed alongside it returns `Some` | CONFIRMED, no reachable divergence -- measured against the actual `DMatrix` allocation ceiling, not asserted | `crates/moveit-planners-stomp/src/cost_functions.rs` `kernel_bounds`; `std::mem::size_of::<f64>()`/`isize::MAX` are the same limits `Vec`'s allocator enforces (`alloc::raw_vec`, capacity check against `isize::MAX` bytes) | `<pending, see report>` |
