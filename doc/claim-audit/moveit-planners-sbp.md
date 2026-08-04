# Claim audit — moveit-planners-sbp

Per `PORTING-PLAN.md` §175: one row per claim, appended as found, not
batched at report time.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-sbp/src/constrained_sampler.rs`, `GroupConstraintSampler`'s "Known gap" doc section | `PlanningRequest::solver` wired into `path_constraints`' `select_default_sampler` call (round 24, `PORTING-PLAN.md` §163.3) makes a Cartesian-constrained *path* region more reliably plannable end-to-end, the way it already does for a Cartesian *goal* region | EXPIRED (claim never actually made this strongly — recorded here because the natural reading of "wire the solver into the path_constraints call site too" implies it, and that implication is false) | Empirical, not an upstream citation: 4 scenarios measured on `panda_arm` (far-apart self-motion position+orientation pair; orientation-only region with a free approach axis; a region built around an already-IK-reachable nearby goal; a `step_size`/iteration-budget sweep for a crossover point) at matched `step_size`/budget between a wired and unwired `PlanningRequest`. No scenario showed wiring reliably improve `solve()`'s success rate; the tightest, most goal-region-analogous scenario measured wired *worse* than unwired (0/5 vs. 5/5 successful solves). Root cause: `GroupConstraintSampler`'s `template` is a fixed per-attempt IK seed, not re-anchored to the tree node being extended from (`constrained_sampler.rs`'s own doc comment, same commit) | (doc-only, no code claim to fix — the actual fix, re-anchoring `template` to tree locality, is deferred, see `constrained_sampler.rs`'s "Disposition" paragraph) |
| `crates/moveit-planners-sbp/src/constrained_sampler.rs`, `GroupConstraintSampler::working` (round 25, the seeding-gap round the row above deferred to) | Replacing the per-attempt `template` clone with a persistent `working: RefCell<RobotState>` -- matching `ompl_interface::ConstrainedSampler`'s/`ConstrainedGoalSampler`'s own never-reset `work_state_` member (`constrained_sampler.cpp`, `constrained_goal_sampler.cpp`) -- makes a wired path sampler reliably beat unwired at the same tight budget the row above measured a *regression* at | EXPIRED (§181: the 1/5-vs-5/5 numbers are real and reproduced, but this row's causal claim about them is not -- see the row below) | `registry::tests::path_constraints_end_to_end_wired_vs_unwired`: at the discriminating budget (`step_size: 0.03`, `goal_bias: 0.0`, `Termination::Iterations(20)`) on `panda_arm`'s self-motion position+orientation pair (0.803693435403718 rad apart, walked via incremental local IK re-solves rather than independent random restarts, which land on disconnected branches 6-7 rad apart), unwired solved 1/5 seeds, wired solved 5/5 -- reproduced identically across 3 repeated runs. Looser budgets (`step_size: 0.2` with `Iterations(200)` or `Iterations(20)`) gave 5/5 for *both* wired and unwired -- not discriminating, since a 0.8 rad separation lets a near-direct connect succeed regardless of sampler at a coarse step size; this is disclosed in the test's own doc comment rather than tuned away. This is one re-measured scenario, not the four-scenario sweep the row above's measurement used (that sweep was never committed as reusable code, so it could not be re-run) | `f8c7af0`, `d853392` |
| `crates/moveit-planners-sbp/src/constrained_sampler.rs`, `GroupConstraintSampler`'s "What `path_constraints_end_to_end_wired_vs_unwired` actually shows" doc section (round 26, correcting the row above) | The row above's causal claim, isolated: does the persistence fix (`GroupConstraintSampler::working`) or the `resolve_constraint_sampler` wiring extension (§163.3, two rows up) actually produce that test's 5/5? | CONFIRMED: the wiring extension, not the persistence fix | Two independent revert experiments against the committed test, each reverted back immediately after measuring. (1) Reverting the persistence fix alone (`working` back to a per-attempt `*state = template.clone()`, reproducing round 24's shape) leaves the numbers unchanged: unwired 1/5, wired 5/5, identical to the digit -- confirms the coordinator's own §181.1 finding independently. Exactly one test reddens crate-wide (`try_sample_carries_the_previous_draws_result_forward_as_the_next_seed`), proving the mutation is real and guarded, just not what this test measures. (2) Reverting instead `resolve_constraint_sampler`'s `path_constraints` call site to always pass `None` (round 23's goal-only `.take()` shape, §163.3's wiring extension undone) collapses wired to 1/5, identical to unwired. This is not merely correlational: `path_constraints_end_to_end_wired_vs_unwired` builds every request with `goal: Goal::State(..)`, so `solve()`'s goal branch never calls `select_default_sampler` at all (`Goal::State(_) => None`); with the path call site also forced to `None`, no `select_default_sampler` call anywhere in either request ever sees a solver, making the "wired" and "unwired" requests behaviourally identical *by construction*. The wiring extension is what lets this test discriminate at all | `5f06e84`, `603f3f7` |
| `crates/moveit-planners-sbp/src/registry.rs`, `path_constraints_four_scenario_wired_vs_unwired_sweep` (round 28, rebuilding row 8's uncommitted sweep per `PORTING-PLAN.md` §187) | Row 8's four-scenario sweep (never committed) measured **wired 0/5 vs unwired 5/5** -- the opposite direction from round 26's committed **unwired 1/5, wired 5/5**. Does that direction reproduce in a committed, independently varied four-scenario sweep? Separately: row 10's experiment 2 (reverting §163.3's wiring collapses wired to unwired) is, by its own admission, close to tautological for a `Goal::State` scenario -- does a *non-tautological* case show the same wiring producing a real benefit, or none? | Round 24's direction does NOT reproduce in any of 6 measured configurations (4 scenarios, 3 of them budget-varied) -- every one has wired >= unwired, most strictly greater. Row 8's number is either scenario-specific to a case none of these four recreate, or was wrong; this sweep cannot tell which without row 8's own scenario, which no longer exists as code. On the tautology question: scenario 3 (orientation-only corridor, free position) has the *same* structural necessity as scenarios 1/4 (`Goal::State`, so solver only ever reaches `path_constraints`) yet measures unwired 5/5, wired 5/5 -- no benefit at all. That the identical "solver must reach `path_constraints` to matter" structure produces a real benefit in scenarios 1/2/4 and *none* in scenario 3 shows the benefit is a genuine, scenario-dependent empirical outcome of the search actually using the sampler's proposals, not a guarantee built into the wiring's presence | Scenario 1 (self-motion, `Goal::State`): unwired 1/5, wired 5/5. Scenario 2 (`Goal::Constraints`, no `path_constraints` -- the non-`Goal::State` case): unwired 0/5, wired 5/5. Scenario 3 (orientation-only corridor, free position): unwired 5/5, wired 5/5. Scenario 4 (budget crossover, scenario 1's geometry): tight (`0.03`/`Iterations(20)`) unwired 1/5 wired 5/5; medium (`0.1`/`Iterations(20)`) unwired 0/5 wired 5/5; loose (`0.2`/`Iterations(200)`) unwired 0/5 wired 5/5. All deterministic, reproduced across repeated runs | `e1f2b67` |
| `crates/moveit-planners-sbp/src/registry.rs`, `path_constraints_end_to_end_wired_vs_unwired`'s own doc comment (pre-round-28 text, found stale while building the row above) | "A looser budget, e.g. `step_size: 0.2`/`Iterations(20)` ... let *both* wired and unwired solve 5/5" -- i.e. the loose end of the budget range is claimed non-discriminating for this test's exact scenario | EXPIRED (wrong): the claim does not hold for the scenario actually committed | Checked directly, not inferred: `small_budget` in the committed `path_constraints_end_to_end_wired_vs_unwired` was temporarily edited to `step_size: 0.2` and re-run at both `Iterations(200)` and `Iterations(20)` (each reverted immediately after measuring). Both measure **unwired 0/5, wired 5/5** -- still discriminating, not tied -- confirmed by the self-motion distance printed alongside (`0.803693435403718 rad`, identical to the tight-budget run, proving the same geometry was under test). This claim was never re-checked against the scenario it was written about before being committed -- the same uncommitted-measurement failure `PORTING-PLAN.md` §187 records for row 8, just smaller in scope and caught this round while building the sweep above rather than by a separate audit | `9d2e511` |

| `crates/moveit-planners-sbp/src/registry.rs`, `path_constraints_four_scenario_wired_vs_unwired_sweep`'s own "# Measured" table (round 29, `PORTING-PLAN.md` §195) | The six numbers in row 11's own evidence and in this test's doc comment were only ever `eprintln!`'d, never asserted -- do they stay true under a change that deletes the effect under measurement? | CONFIRMED the gap was real, then closed | Probed first: routing `run_scenario`'s wired branch to `solver: None` (deleting the whole effect this sweep measures) left the test green. Added `assert_eq!` pinning all six numbers exactly (seeds are `ChaCha8Rng`-deterministic, including scenario 3's `(5, 5)` tie, itself a finding). Re-ran the same probe after: it now reddens on scenario 1's mismatch (`left: (1, 1)`, `right: (1, 5)`), matching the coordinator's own independent re-run of the same probe | `bda9e2e`, `494b4dc` |
| `crates/moveit-planners-sbp/src/registry.rs`, `path_constraint_sampler_is_load_bearing_not_merely_invoked`'s doc comment (round 29, `PORTING-PLAN.md` §195 anchor sweep) | "scored 30/30 unwired failures *and* 30/30 wired successes across seeds `0..30` -- see this round's git history for the sweep" | Was UNVERIFIABLE (the committed test ran exactly one seed, not 30; the cited sweep was never committed code) -- now CONFIRMED | Test body looped over `0..30u64`, both branches, with per-seed `assert_eq!`/`unwrap_or_else(panic!)`; passes at all 30 seeds, 0.13s. Same defect family as row 8's original uncommitted sweep, found in a test that predates that round's work | `82fc3f8` |
| `crates/moveit-planners-sbp/src/registry.rs`, `path_constraints_end_to_end_wired_vs_unwired`'s own "Measured after the fix: unwired 1/5, wired 5/5" doc claim (round 29, §195) | Backed only by `wired_successes > unwired_successes`, silently satisfiable by unwired 2, 3, or 4/5, not just the documented 1/5 | CONFIRMED exactly | `assert_eq!((unwired_successes, wired_successes), (1, 5))`, deterministic seeds; passes | `36eb630` |
| `crates/moveit-planners-sbp/src/goal_sampler.rs`, `constrained_branch_is_load_bearing_not_merely_invoked`'s 5-row window/budget table (round 29, §195 anchor sweep) | 4 of 5 rows (every row but the selected `(0.01, 5)`) were prose citing a sweep with no committed code behind it at all | Was UNVERIFIABLE for 4/5 rows -- now CONFIRMED for all 5 | New test `window_budget_sweep_matches_the_documented_table` re-measures all five `(window, budget)` pairs, 30 seeds each, `assert_eq!` per row; passes, 0.44s | `e370581` |

## §195 anchor sweep (round 29)

`PORTING-PLAN.md` §195: a number cited in a doc comment or this audit
file's own evidence column is a claim, not an observation, once
something depends on it — "is it cited anywhere," not "is it a
regression," is the line for whether it needs a gate.

**Mechanical anchor:** `rg -n 'eprintln!|println!' crates/moveit-planners-sbp/{src,tests,examples} crates/moveit-constraints/{src,tests}`.
3 hits, all inside `crates/moveit-planners-sbp/src/registry.rs`
(`path_constraints_end_to_end_wired_vs_unwired`,
`path_constraints_four_scenario_wired_vs_unwired_sweep`) — both closed
by the rows above.

**Where the mechanical anchor stops (`PORTING-PLAN.md` §191):**
`eprintln!`/`println!` only catches a value that is *printed*. It
cannot catch a value that is computed and asserted with a check weaker
than the number cited elsewhere for it (`path_constraints_end_to_end_wired_vs_unwired`'s
own `unwired_successes` was printed, but the print was not the defect —
the `>` assertion below it was), nor a documented number attached to a
*different* test than the one that would compute it, nor a number
citing a sweep with no code in this crate at all
(`path_constraint_sampler_is_load_bearing_not_merely_invoked`'s "see
this round's git history," `constrained_branch_is_load_bearing_not_merely_invoked`'s
4 non-selected table rows — neither test printed anything to find by
grepping for a print macro). Closing those needed hand enumeration:
`rg -n '^\s*///.*\d+/\d+' crates/moveit-planners-sbp/src crates/moveit-constraints/src`
(plus a second pass for prose without a `/` — "always", "reliably",
"100%", "measured", "swept" — to catch claims not shaped as a ratio),
then reading each hit's owning test to judge whether its assertion
matched the citation. This does not reduce to a single command; the
full classified list is below.

| site | classification | disposition |
|---|---|---|
| `registry.rs::path_constraints_four_scenario_wired_vs_unwired_sweep` | same defect | fixed, `bda9e2e`/`494b4dc` (row above) |
| `registry.rs::path_constraint_sampler_is_load_bearing_not_merely_invoked` | same defect, more severe (claimed sample size did not match the seed actually run) | fixed, `82fc3f8` (row above) |
| `registry.rs::path_constraints_end_to_end_wired_vs_unwired` | same defect (weaker-than-cited inequality) | fixed, `36eb630` (row above) |
| `registry.rs::solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable`'s "0/10" | distinct — already loops `0..10` with a hard `assert!`/`matches!` per seed on both branches; the doc's "0/10" is exactly what the loop enforces | no action |
| `goal_sampler.rs::constrained_branch_is_load_bearing_not_merely_invoked`'s table, selected row `(0.01, 5)` | distinct — already reproduced exactly by that test's own `0..30` loop | no action |
| `goal_sampler.rs::constrained_branch_is_load_bearing_not_merely_invoked`'s table, other 4 rows | same defect | fixed, `e370581` (row above) |
| `constrained_sampler.rs`'s "5/5"/"1/5" narrative mentions | distinct — citations of `registry.rs` test numbers that are themselves now exactly asserted (rows above); not an independent, separately-computed claim | no action |
| `planning_scene_validity.rs`'s `~8-15 ms`/`~1-6 ms` timing figures | distinct — wall-clock, not `ChaCha8Rng`-deterministic; the module's own doc comment already discloses the guard test's bound is deliberately loose (~100x margin) and points to `examples/planning_scene_validity_bench.rs` for a precise re-measurement. Exact-pinning a wall-clock figure would trade a real defect (an unasserted claim) for a worse one (a flaky test); the loose bound is the correct choice, not an instance of this defect | no action |
| `space.rs::sample_near_is_uniform_over_the_ball_volume` | distinct — Monte Carlo estimate around an analytically-derived expected value (`0.5^3`), already asserted with a numeric tolerance (`< 0.02`) sized to sampling variance, not a deterministic count standing in for an exact one | no action |
| `moveit-constraints/src/visibility.rs`'s rounds 15-23 pr2 oracle-mismatch narrative (105/115, 10/115, 43/105, 4/14, 0/2400, etc.) | reviewed, out of this round's scope | see below |

**`visibility.rs` explicitly excluded, not silently skipped:** this is a
different, already-multi-round-audited investigation (its own text
cites `PORTING-PLAN.md` §148 and other panels by name — "p1-joints",
"p3-acm" — not this round's work), and its live conclusion (`0/2400`
`touching >= 2`, §169 in this repo's shared `PORTING-PLAN.md`) is
already backed by its own separately committed, independently runnable
tooling (`cargo run --release --example visibility_cone_depth_sweep -p
moveit-constraints`, and an `#[ignore]`d
`visibility_cone_ambiguity_diagnostic` test), not a `#[test]` in this
crate's own suite silently citing an unreproducible number. Re-auditing
23 rounds of a different panel's oracle-parity history is outside what
this round's §195 was scoped to find; flagged here so the exclusion is
a decision, not an omission.

`crates/moveit-constraints/tests/*.rs` (`decide.rs`, `utils_parity.rs`,
`ik_sampler.rs`, `constraint_sampler_manager.rs`, `sampler.rs`,
`orientation_accessors.rs`) and `crates/moveit-planners-sbp/tests/plan_space_parity.rs`
had zero hits on both passes.

## Notes

This crate's audit history through round 23 lives in
`doc/d12-solver-none-structural-measurement.md` (D12's `KinematicsSolver`
crate-layering measurement, not a claim-audit-table format — predates
§175's convention). This file starts fresh under the §175 table format
rather than retroactively reformatting that report.
