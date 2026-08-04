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
