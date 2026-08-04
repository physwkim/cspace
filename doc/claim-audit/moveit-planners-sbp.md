# Claim audit — moveit-planners-sbp

Per `PORTING-PLAN.md` §175: one row per claim, appended as found, not
batched at report time.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-planners-sbp/src/constrained_sampler.rs`, `GroupConstraintSampler`'s "Known gap" doc section | `PlanningRequest::solver` wired into `path_constraints`' `select_default_sampler` call (round 24, `PORTING-PLAN.md` §163.3) makes a Cartesian-constrained *path* region more reliably plannable end-to-end, the way it already does for a Cartesian *goal* region | EXPIRED (claim never actually made this strongly — recorded here because the natural reading of "wire the solver into the path_constraints call site too" implies it, and that implication is false) | Empirical, not an upstream citation: 4 scenarios measured on `panda_arm` (far-apart self-motion position+orientation pair; orientation-only region with a free approach axis; a region built around an already-IK-reachable nearby goal; a `step_size`/iteration-budget sweep for a crossover point) at matched `step_size`/budget between a wired and unwired `PlanningRequest`. No scenario showed wiring reliably improve `solve()`'s success rate; the tightest, most goal-region-analogous scenario measured wired *worse* than unwired (0/5 vs. 5/5 successful solves). Root cause: `GroupConstraintSampler`'s `template` is a fixed per-attempt IK seed, not re-anchored to the tree node being extended from (`constrained_sampler.rs`'s own doc comment, same commit) | (doc-only, no code claim to fix — the actual fix, re-anchoring `template` to tree locality, is deferred, see `constrained_sampler.rs`'s "Disposition" paragraph) |

## Notes

This crate's audit history through round 23 lives in
`doc/d12-solver-none-structural-measurement.md` (D12's `KinematicsSolver`
crate-layering measurement, not a claim-audit-table format — predates
§175's convention). This file starts fresh under the §175 table format
rather than retroactively reformatting that report.
