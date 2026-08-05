# Claim audit: moveit-stomp-core

Per PORTING-PLAN.md §175. One row per item, appended as found, not
batched at report time. `evidence` is the upstream `file:line` actually
opened this round -- inference from the port is not evidence.

Upstream root for this crate: `/home/stevek/work/stomp` (pinned
`b1a87c80f7338caae25a5c689b876da15492aa75`, `ros-industrial/stomp`).

## §172 sweep (round 33), upstream-first per `0ad8c67`

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `/home/stevek/work/stomp/src/utils.cpp:39` (`(int)order`) | `order` is `DerivativeOrder`-equivalent (an enum), cast to `int` for `pow(dt, (int)order)` -- integer/enum-derived, not real-valued | CONFIRMED distinct | `/home/stevek/work/stomp/src/utils.cpp:39` -- `double multiplier = 1.0 / pow(dt, (int)order);` | (none) |
| `/home/stevek/work/stomp/{include/stomp/*.h,src/*.cpp}` full sweep | No float-derived `int`/`unsigned`/`size_t`/`long` declaration or `static_cast` anywhere else in this crate's upstream | CONFIRMED, 0 additional hits | Full-file grep of `stomp.h`, `task.h`, `utils.h`, `stomp.cpp`, `utils.cpp`; all `int`/`size_t`/`unsigned` declarations found (`stomp.cpp:117-119,158,231,303,435-480`; `stomp/src/utils.cpp:44,66-67,113-115`) are `.rows()`/`.cols()`/`.size()`/config-count-derived | (none) |
| `crates/moveit-stomp-core/src/{stomp,utils}.rs` (port-side anchor: `as i8..u128/usize` receiving `f64`) | All 12 hits narrow an enum (`DerivativeOrder`), a loop/index variable, or an integer count -- none are real-valued | CONFIRMED distinct, 12 sites: `stomp.rs:550,792,817,878`; `crates/moveit-stomp-core/src/utils.rs:330-332,339-340,421-422,424,543,555` | Read in this tree only | (none) |

## §172 row 3 (round 33) re-verified with the full anchor enumeration, per review

The round-33 row above asserts "12 hits, none real-valued" without
showing the enumeration. Re-run here with the anchor regex on screen,
because the count itself was off (16 casts across 14 lines, not 12 --
the underlying "none float-derived" conclusion is unchanged, but it
was not actually a 12-site claim):

**Anchor:** `` as (i8|i16|i32|i64|i128|u8|u16|u32|u64|u128|usize|isize)\b `` (`rg`, `crates/moveit-stomp-core/src/`)

**Sites** (16 casts, 14 lines):

| site | expression | source | classification |
|---|---|---|---|
| `crates/moveit-stomp-core/src/utils.rs:330` | `order as i32` | `DerivativeOrder` enum param | enum, not float |
| `crates/moveit-stomp-core/src/utils.rs:331` | `(FINITE_DIFF_RULE_LENGTH / 2) as isize` | `FINITE_DIFF_RULE_LENGTH: usize = 7` (`crates/moveit-stomp-core/src/utils.rs:253`) | integer constant, not float |
| `crates/moveit-stomp-core/src/utils.rs:332` | `num_time_steps as isize` | `num_time_steps: usize` fn param | integer param, not float |
| `crates/moveit-stomp-core/src/utils.rs:339` | `i as usize` | `i: isize` loop var (`0..n`, `n: isize`) | integer loop index, not float |
| `crates/moveit-stomp-core/src/utils.rs:339` | `index as usize` | `index = i + j` (`isize` arithmetic) | integer arithmetic, not float |
| `crates/moveit-stomp-core/src/utils.rs:340` | `order as usize` | `DerivativeOrder` enum param | enum, not float |
| `crates/moveit-stomp-core/src/utils.rs:340` | `(j + half) as usize` | `j`, `half: isize` loop vars | integer arithmetic, not float |
| `crates/moveit-stomp-core/src/utils.rs:421` | `order as usize` | `DerivativeOrder` enum param | enum, not float |
| `crates/moveit-stomp-core/src/utils.rs:422` | `order as usize` | `DerivativeOrder` enum param | enum, not float |
| `crates/moveit-stomp-core/src/utils.rs:424` | `order as i32` | `DerivativeOrder` enum param | enum, not float |
| `crates/moveit-stomp-core/src/utils.rs:543` | `DerivativeOrder::Velocity as usize` | enum literal (test) | enum, not float |
| `crates/moveit-stomp-core/src/utils.rs:555` | `DerivativeOrder::Acceleration as usize` | enum literal (test) | enum, not float |
| `stomp.rs:550` | `self.current_iteration as usize` | `current_iteration: i32` field | integer field, not float |
| `stomp.rs:792` | `r as i32` | `r: usize` loop var (`0..num_rollouts`) | integer loop index, not float |
| `stomp.rs:817` | `r as i32` | `r: usize` loop var | integer loop index, not float |
| `stomp.rs:878` | `r as i32` | `r: usize` loop var | integer loop index, not float |

**Same defect at:** none -- zero of the 16 casts narrow a value derived
from an `f64` computation.

**Distinct, skip:** all 16 -- every source is an enum discriminant, an
already-integer field/param, or an integer loop index. None of the
`kernel_bounds`-style divergence applies here at all: that divergence
is specifically float-to-int narrowing (Rust's `as` saturates,
C++'s `static_cast` is UB on overflow); every cast in this crate is
int-to-int or enum-to-int, where both languages truncate identically
and no divergence class exists to test a boundary against.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-stomp-core/src/{stomp,utils}.rs` full anchor re-enumeration | 16 casts (not 12), 0 float-derived; no boundary test is owed because int-to-int/enum-to-int narrowing has no Rust/C++ divergence class to begin with | CONFIRMED distinct, 16/16 sites enumerated and classified above | `rg` anchor search + type of every cast's source read in this tree | `3a0e278` |

## §167.6 bare-directory-citation sweep (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| Every `Ported from` header in this crate (`crates/moveit-stomp-core/src/lib.rs:5`, `task.rs:5`, `stomp.rs:5`, `crates/moveit-stomp-core/src/utils.rs:5`) | None cite a bare package/directory line with no filenames indented beneath it -- every citation lists explicit `include/stomp/*.h`/`src/*.cpp` files | CONFIRMED, 0 hits of the shape the parser now closes | Read all four headers in full in this tree; `tools/ci/verify-upstream-license-provenance.sh` also run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |

## `create_3dof_configuration` sweep, per review of `788cc3f`

`num_iterations_after_valid: 0` in `test::create_3dof_configuration`
masked two cancellation assertions across two separate rounds
(`cancelling_from_another_thread_stops_a_plan_call_already_in_flight`,
round 34, a different crate/helper --
`moveit-planners-stomp::planner::base_config` -- not this one;
`cancelling_before_solve_stops_before_num_iterations_completes`, this
round, `788cc3f`). Reviewer asked whether more than these two call
sites of *this* helper are similarly sensitive, in which case the
helper's default itself would be the structural fix rather than
patching callers one at a time. Full enumeration, all 9 call sites:

**Anchor:** `create_3dof_configuration(` (`rg`,
`crates/moveit-stomp-core/src/stomp.rs`)

| test | calls `.solve`/`.solve_from_endpoints`? | asserts on iteration count or cancellation? | classification |
|---|---|---|---|
| `construction_does_not_panic` | no | no | distinct -- never solves, `num_iterations_after_valid` cannot matter |
| `solve_default_converges_to_the_bias_trajectory_from_endpoints` | yes | no -- asserts `compare_diff` (converged within `BIAS_THRESHOLD`) | distinct -- *wants* the early-valid break; that break is what "converges promptly" means here |
| `solve_with_linear_interpolated_initial_trajectory_converges` | yes | no -- `compare_diff` only | distinct, same reason |
| `solve_with_cubic_polynomial_initial_trajectory_converges` | yes | no -- `compare_diff` only | distinct, same reason |
| `solve_with_minimum_control_cost_initial_trajectory_converges` | yes | no -- `compare_diff` only | distinct, same reason |
| `solve_with_40_timesteps_converges` | yes | no -- `compare_diff` only | distinct, same reason |
| `solve_with_60_timesteps_converges` | yes | no -- `compare_diff` only | distinct, same reason |
| `cancelling_before_solve_stops_before_num_iterations_completes` | yes | yes | **cancellation-sensitive -- fixed in `788cc3f`** (overrides `num_iterations_after_valid = num_iterations` on its own local `config`) |
| `second_solve_call_ignores_its_initial_parameters_argument` | yes (twice) | no -- asserts non-reseeding, not iteration count; also sets `num_iterations = 1`, which already bounds the loop tighter than the early-valid break could | distinct |

**Same defect at:** none beyond the one already fixed.

**Distinct, skip:** 7 of 9 -- six convergence tests whose own
assertion (`compare_diff` against a bias trajectory) is the behavior
`num_iterations_after_valid: 0` exists to produce, not something it
masks; one reseeding test that neither reads iteration count nor
lets the early-valid break matter (`num_iterations = 1` already caps
it tighter).

**Conclusion:** exactly 1 of this helper's 9 call sites was ever
cancellation-sensitive, and it is now fixed locally. Per the
reviewer's own stated conditional -- change the default only if more
than the two cited tests turn out sensitive -- the count came back
lower, not higher: only one call site of *this* helper needed the
override, and the other two known-sensitive tests fixed across both
rounds (round 34, this round) each live in different
crates/fixtures anyway. Changing the default to a value that removes
the early-valid break would only break the majority use: 7 of 9
sites use `num_iterations_after_valid: 0` to test genuine "stops
promptly once valid" behavior, and forcing every one of them to
either set a very high `num_iterations_after_valid` themselves or
accept a materially different (higher iteration count, longer)
convergence test would be a change with no corresponding defect to
justify it. The per-caller override applied in `788cc3f` is the
correct fix, not a patch standing in for a deferred structural one.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-stomp-core/src/stomp.rs`, all 9 `create_3dof_configuration` call sites | Only 1 of 9 is cancellation/iteration-count sensitive; the helper's default should not change | CONFIRMED, 9/9 sites enumerated and classified above | Read every call site's body in this tree | (pending, see report) |

## Mutation-probe evidence for the `create_3dof_configuration` sweep, per §201 (this round)

The sweep above (rows 89-94) classified six convergence tests as
"distinct -- *wants* the early-valid break" by reading the source, with
no measurement backing that reading. §201: a classification that
decides against a structural fix is a claim, and claims need
reproducible evidence, not a one-time reasoning pass. The coordinator
measured it by hand first -- disabling `update_parameters`'s
accept-path accumulation (`self.parameters_optimized +=
&self.parameters_updates;`, `stomp.rs:1049`) and re-running this
crate's tests -- and found it fails five of the six:
`solve_default_converges_to_the_bias_trajectory_from_endpoints`,
`solve_with_linear_interpolated_initial_trajectory_converges`,
`solve_with_cubic_polynomial_initial_trajectory_converges`,
`solve_with_minimum_control_cost_initial_trajectory_converges`,
`solve_with_40_timesteps_converges`. This worker reproduced that result
independently and then extended it to all 9 sites, which surfaced a
sixth case the coordinator's report did not mention either way:
`solve_with_60_timesteps_converges` does **not** fail under the same
mutation.

That reproduction is now permanent and re-runnable, not a one-time
hand edit: `Stomp::disable_accept_update_for_test`
(`crates/moveit-stomp-core/src/stomp.rs`, a `#[cfg(test)]`-only field,
absent from release builds) skips exactly the accept-path `+=`, the
same single line disabled by hand. Two tests exercise it --
`stomp::tests::each_convergence_test_fails_if_the_accept_path_update_is_disabled`
(the five confirmed-sensitive scenarios, each asserted to fail with the
line disabled) and
`stomp::tests::solve_with_60_timesteps_converges_is_a_known_gap_in_this_probe`
(the sixth scenario, asserted to *still pass* with the line disabled,
pinning the gap rather than hiding it). Both tests carry the full
mechanism in their own doc comments; not restated here beyond the
summary below.

Root cause of the probe's differential coverage, traced with temporary
`eprintln!` instrumentation on `solve`/`compute_optimized_cost`
(removed before this commit): every one of these six tests, mutated or
not, runs its optimization loop **exactly once**. `create_3dof_configuration`'s
`num_iterations_after_valid: 0` breaks the loop as soon as one valid
iteration is seen, and the seed itself (from `compute_initial_trajectory`,
before any rollout/update touches it) is already `parameters_valid` at
the pre-loop cost check for every initialization method these tests
use -- confirmed by counting `compute_optimized_cost` calls per test:
exactly two (the pre-loop call, then one real loop iteration). Counted
directly for both conditions, not inferred for one of them: the
unmutated count came from the six production tests themselves
(`cargo nextest run -p moveit-stomp-core` with a temporary per-call
`eprintln!`); the mutated count came from running
`each_convergence_test_fails_if_the_accept_path_update_is_disabled`
(covers five of the six) and
`solve_with_60_timesteps_converges_is_a_known_gap_in_this_probe`
(the sixth) the same way, with a temporary scenario marker between
the five inline scenarios of the first so each could be counted on
its own -- two calls, no more, in every one of the twelve
(six tests x two conditions) runs. `num_iterations`/`num_iterations_after_valid`
overrides of 40 or 100 on several of these tests are therefore inert:
none of them ever run more than one real iteration regardless. That
single iteration's reject-branch (`compute_optimized_cost`'s "cost did
not improve" path) always fires on these six unmutated too, and its
`+=` then `-=` of the same, unmodified `parameters_updates` cancel
exactly -- which is why the *unmutated* tests converge trivially (the
seed alone already satisfies `BIAS_THRESHOLD`) rather than because many
iterations of real optimization ran. Disabling only the `+=` breaks
that cancellation: `parameters_optimized` ends up shifted from the seed
by exactly `parameters_updates`, and whether that shift's per-element
magnitude clears `BIAS_THRESHOLD` (0.05) depends on the fixed
`ChaCha8Rng` seed (`DummyTask::new`'s `seed: u64` -- 5 for the
40-timestep test, 6 for the 60-timestep test) and on how many columns
dilute it (40 vs 60). Measured: `update_norm` (Frobenius norm of
`parameters_updates`) was ~0.615 for the 40-timestep/seed-5 case
(fails) and ~0.198 for the 60-timestep/seed-6 case (passes). This is
coincidental to the fixed seed, not a difference in whether real
optimization is exercised -- both scenarios depend on the identical
mechanism, they just land on opposite sides of the threshold for this
particular seed pair.

**What the original sweep's classification still gets right:** all six
tests' `compare_diff` assertions genuinely depend on the seed landing
close to the bias trajectory, and `num_iterations_after_valid: 0`
existing to produce that promptly (not masking anything) is confirmed,
not just asserted, for five of six. **What it does not establish:** that
this dependency is *reliably measurable* by this one mutation for the
sixth (`solve_with_60_timesteps_converges`) -- that is weak, seed-lucky
coverage, honestly left open rather than papered over by picking a
different seed to force a fail.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-stomp-core/src/stomp.rs`, six `compare_diff`-asserting `create_3dof_configuration` sites (sweep rows 89-94) | Five genuinely depend on `update_parameters`'s accept-path accumulation, reproducibly; the sixth (`solve_with_60_timesteps_converges`) does not reliably, for a measured, seed-dependent reason, not a defect in that test | CONFIRMED for 5/6 (permanent regression tests), OPEN/known gap for 1/6 (`solve_with_60_timesteps_converges`), both pinned by tests rather than left in chat | `stomp::tests::each_convergence_test_fails_if_the_accept_path_update_is_disabled`, `stomp::tests::solve_with_60_timesteps_converges_is_a_known_gap_in_this_probe`, both run under `cargo nextest run -p moveit-stomp-core` | (pending, see report) |

## §194 port-only API sweep (this round): `moveit-stomp-core`

Reviewer's ask, after `a682f63` confirmed `Stomp::with_cancel_handle`
broke an upstream invariant (`resetVariables`'s `proceed_ = true`
assumed no caller could reach `proceed_` before construction --
true in upstream, false once this port added a pre-construction
handle) that a code-level transcription check could not have caught:
enumerate every public constructor/setter/handle-accepting API in
this crate with no upstream counterpart, and judge whether each one
makes any *other* upstream-private, upstream-invariant-protected
state newly reachable.

**Anchor:** every `pub fn`/`pub struct` in `crates/moveit-stomp-core/src/{stomp,task,utils}.rs`
(`rg '^\s*pub fn|^\s*pub struct'`), cross-referenced against
`/home/stevek/work/stomp` for an upstream counterpart.

| API | port-only? | upstream state newly reachable | invariant at risk? |
|---|---|---|---|
| `CancelHandle::new()` | yes (round 24) -- upstream's `proceed_` is a private `Stomp` member; nothing upstream can construct a handle to it before a `Stomp` exists | a cancellation flag, settable before any `Stomp` exists | **yes -- confirmed, fixed in `a682f63`** (see `Stomp::with_cancel_handle` below) |
| `CancelHandle::cancel(&self)` | yes, but only as a method on the port-only handle type above -- the flag it stores into is the same one `Stomp::cancel()` stores into in upstream, no new state of its own | none beyond `CancelHandle::new()`'s | no, covered by the row above |
| `Stomp::with_cancel_handle(config, task, cancel_handle)` | yes (round 24) -- upstream's constructor takes only `(config, task)`; this port's second constructor is the entry point that let a pre-cancelled `CancelHandle::new()` reach `resetVariables`'s unconditional `proceed_ = true` | the already-flagged-false `proceed` state, at construction time | **yes -- confirmed, fixed in `a682f63`** |
| `Stomp::cancel_handle(&self)` (obtain a handle *after* construction) | yes -- upstream has no way to hand a `proceed_`-sharing handle to another thread; only `stomp->cancel()` from whoever holds `&Stomp` | a clone of the same `Arc<AtomicBool>` `Stomp::cancel()` already writes | no -- mirrors upstream's own established contract exactly: upstream's own `stomp_moveit_planning_context.cpp:247-257` watcher thread already calls `stomp->cancel()` concurrently with `solve()` running on another thread, so `proceed_` being externally, concurrently settable while `solve()` runs is upstream-intended behavior, not new state this port opened up; `Arc<AtomicBool>` gives the identical thread-safety property `std::atomic<bool>` already provided |
| `Stomp::new(config, task)` | no -- matches upstream's constructor signature; routes through `with_cancel_handle` with `CancelHandle::new()`, which starts at `true`, same as upstream's own `resetVariables`-driven `proceed_ = true` at construction | none new -- behaviorally identical to upstream both before and after `a682f63`'s fix | no |
| `Stomp::set_config`/`Stomp::clear` | no -- direct ports of upstream `setConfig`/`clear`, both routing through `resetVariables` in upstream too | none new; `a682f63` made the `proceed = true` reset *explicit* in these two rather than implicit in every `resetVariables` caller, matching upstream's actual intent more precisely, not granting new reach | no |

**Considered and rejected as a finding:** a `CancelHandle` obtained
via `Stomp::cancel_handle()` and held by a background thread can
race, at the `AtomicBool` level, against a same-instant
`Stomp::clear()`/`Stomp::set_config()` call resetting `proceed` back
to `true` from the owning thread. Not treated as a defect: upstream
itself has the analogous ambiguity for concurrent `clear()`/`cancel()`
already (`config_`, `parameters_optimized_` etc. are plain,
non-atomic members with no documented thread-safety contract against
a concurrent `clear()`/`setConfig()`), so no upstream invariant
depended on this being unreachable -- it already wasn't guaranteed
safe upstream, and `Arc<AtomicBool>`'s "last store wins" is
well-defined, not a soundness gap, just an ambiguity upstream never
resolved either.

**Conclusion:** `Stomp::with_cancel_handle` (via `CancelHandle::new()`)
is the only port-only API in this crate that made an upstream
invariant-protected state reachable that upstream's own design kept
structurally unreachable, and it is now fixed. Every other port-only
surface in this crate either carries no new capability or mirrors an
upstream-established concurrency contract exactly.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-stomp-core/src/stomp.rs`, every `pub fn`/`pub struct` | `with_cancel_handle` is the only port-only entry point that broke an upstream invariant; the rest carry no new risk | CONFIRMED, 6 APIs enumerated and classified above | Read every public item in this tree, cross-referenced against `/home/stevek/work/stomp/include/stomp/stomp.h`, `src/stomp.cpp` | (pending, see report) |
