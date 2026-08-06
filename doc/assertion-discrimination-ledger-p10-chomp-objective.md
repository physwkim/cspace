# Assertion-discrimination ledger — CHOMP objective (`moveit-planners-chomp`)

Produced by p10-phase5 while making CHOMP's objective function observable
(`ChompObjective`/`ChompObjectiveProgress`, `ChompOptimizer::objective`,
`ChompSolution::objective`). It classifies the one coarse-assertion site
that round added; it does **not** re-open this crate's existing rows, which
live in `doc/assertion-discrimination-ledger-p1-robotmodel.md`,
`doc/assertion-discrimination-ledger-p3-acm.md` and
`doc/assertion-discrimination-ledger-p9-ros.md`. A separate file rather than
a row appended to any of those, for the reason
`reconcile-assertion-ledgers.py`'s `discover_ledgers` records and
`doc/assertion-discrimination-ledger-cached-ik.md` restates: ledgers are
globbed, panels own their own file, and appending to another panel's file
across worktrees only manufactures merge conflicts.

The round's other new tests compare exact values
(`assert_relative_eq!` against an independently computed expectation, or an
ordering between two measured totals), so they are not coarse-assertion
sites and need no rows. The one exception below is
`assert_eq!(optimizer.objective(), None)`, kept in that shape because the
whole point of the field being `Option` is that "no iteration evaluated the
objective" is *not* a number; rewriting the assertion to dodge the scanner
would be gaming the gate rather than accounting to it.

Verdict legend (same as the other ledgers): **discriminating** (proven to
distinguish ≥2 sibling outcomes, by bite, isolating fixture design, or
structural single-code argument), **single-branch** (exactly one possible
construction site reaches this assertion — nothing to discriminate),
**joint-collapse** (≥2 real sibling guards fire together at this assertion's
fixture and it cannot say which), **not-this-family** (excluded by one of
census §9's three clauses).

Evidence legend: **bite** (a fresh reachability mutation run this round,
reverted after confirming), **read** (a structural argument from the source,
stated so it can be checked).

## `optimizer.rs` (1)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-planners-chomp/src/optimizer.rs:2206` | `ChompOptimizer::objective`'s `Option` when `optimize`'s loop body never ran | `objective_is_none_when_no_iteration_ever_evaluated_it` | single-branch | read + two bites, below |

**read.** `objective()` returns `self.best_objective` verbatim, and that
field has exactly two writers: the constructor's `None` initializer, and the
`None => { … Some(ChompObjectiveProgress { … }) }` arm at the top of
`optimize`'s loop body, which is the only expression in the crate that
produces `Some` here. `None` therefore means one thing and only one thing —
the loop body did not execute — and there is no sibling `None`-producing
path for the assertion to be blind to. That is the property the `Option`
was chosen for: the value it replaced (`best_group_trajectory_cost: f64`,
initialised to `0.0`) had two meanings, "not measured" and "measured as
zero", and a test asserting `0.0` could not tell them apart.

**bite 1 (run this round, reverted): the assertion is reachable and can
fail.** `optimize`'s loop bound `while self.iteration <
self.parameters.max_iterations` mutated to `<=`, so `max_iterations: 0`
executes one pass: this test fails (`Some(..)` where `None` was asserted),
along with `optimize_runs_exactly_max_iterations_when_filter_mode_and_mesh_to_mesh_never_break_out`
and `optimize_collision_threshold_break_is_a_strict_less_than` — 88 passed,
3 failed. Reverted; `md5sum` against the pre-bite copy matched afterwards.

**bite 2 (run this round, reverted): what this assertion is blind to, and
which test covers it instead.** `optimize` clears `best_objective` on entry
so a second call does not inherit the first call's `seed` — the port's
stand-in for upstream re-seeding `best_group_trajectory_cost_` through its
`iteration_ == 0` arm (`chomp_optimizer.cpp:332-337`). Deleting that
`self.best_objective = None;` line leaves *this* assertion passing (one
call, so nothing to inherit) and fails exactly one test in the crate,
`a_second_optimize_reseeds_the_objective_from_the_first_calls_result` — 90
passed, 1 failed. The blind spot is real, it is named here, and it is
closed by that second test rather than by widening this one. Reverted;
`md5sum` against the pre-bite copy matched afterwards.
