# Assertion-discrimination ledger — p1-robotmodel, round 8

Per-site record for the 55 sites assigned this round, against the
workspace census (`assertion-discrimination-census.md`, merged as
`47d3475`). Enumerated with a hand-written paren-depth/comment-masking
scanner replicating the census's exact method (mask `//`/`/* */`
comments, string/char-literal aware; find every `assert!(`, track paren
depth over the unmasked text to the matching close; classify
`matches!` first, then `bare` if the body contains `.is_err()` or
`.is_none()`). Per-crate reconciliation against the user-supplied
census figures:

| crate | census (matches/bare/total) | this scan | agree? |
|---|---:|---:|---|
| `moveit-trajectory` | 0/16/16 | 0/16/16 | yes |
| `moveit-planners-chomp` | 8/6/14 | 8/6/14 | yes |
| `moveit-planners-sbp` | 3/4/7 | 3/4/7 | yes |
| `moveit-planners-stomp` | 0/7/7 | 0/7/7 | yes |
| `moveit-planning` | 2/2/4 | 2/2/4 | yes |
| `moveit-sampling` | 0/4/4 | 0/4/4 | yes |
| `moveit-kinematics` | 1/2/3 | 1/2/3 | yes |
| **total** | **14/41/55** | **14/41/55** | **no disagreement** |

No scanner/census disagreement found in any of my 7 crates (unlike the
p3-acm precedent, which found 2 disagreements that cancelled in the
total).

Verdict legend: **discriminating** (proven to distinguish ≥2 sibling
outcomes, by bite or variant-exclusion), **single-branch** (exactly one
possible construction site reaches this assertion — nothing to
discriminate), **fixture-collapse-fixed** (a genuine finding fixed this
round), **not-this-family** (the assertion is not testing error-guard
selection at all — a precondition/round-trip/vacuous check).

Evidence legend: **commit** (a specific SHA whose message addresses
this exact site), **round-report** (a prior round's claim — in a doc
or an in-source comment — re-read this round and independently
verified against the current tree), **bite** (a fresh reachability
and/or isolating mutation run this round, reverted after confirming).

## moveit-trajectory (16)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `time_optimal_trajectory_generation.rs:1043` | `TotgOptions::with_resample_dt` combined guard | `resample_dt_zero_is_rejected_not_hung` | single-branch | commit `52e38a3` (structural: `resample_dt` made `pub(crate)`, settable only through this one validating builder — no other construction path exists) — re-derived this round to the `assert!` line; was line 1038 (a `///` line of the test's own doc comment), and `:1030` before round 17 |
| `time_optimal_trajectory_generation.rs:1056` | same | `resample_dt_negative_is_rejected_not_silently_truncated` | single-branch | commit `52e38a3` — re-derived this round; was line 1043, which round 17 left behind when it corrected only rows 1 and 3 of this block |
| `time_optimal_trajectory_generation.rs:1125` | same | `resample_dt_nan_is_rejected` | single-branch | commit `52e38a3` — re-derived this round to the `assert!` line; was line 1120 (a `///` line), and `:1112` before round 17 |
| `time_optimal_trajectory_generation.rs:1138` | same | `resample_dt_positive_infinity_is_rejected` | single-branch | commit `52e38a3` — re-derived this round; was line 1125, left behind by round 17 as row 2 was |
| `time_optimal_trajectory_generation.rs:1149` | same | `resample_dt_negative_infinity_is_rejected` | single-branch | commit `52e38a3` — re-derived this round; was line 1136 (the `fn` line), left behind by round 17 as rows 2 and 4 were |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:512` | `add_suffix_way_point`'s empty+nonzero-dt guard | `add_suffix_way_point_on_an_empty_trajectory_rejects_a_nonzero_dt` | single-branch | bite: neutralized guard (`if false && ...`) → test failed; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:560` | `set_way_point_duration_from_previous`'s index-0 guard | `set_way_point_duration_from_previous_at_zero_rejects_a_nonzero_value` | single-branch | bite: neutralized guard → test failed; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:578` | `append`'s empty+nonzero-dt guard | `append_onto_an_empty_trajectory_rejects_a_nonzero_dt` | single-branch | bite: neutralized guard → test failed; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:596` | `way_point`'s `.get(index).ok_or_else` | `empty_trajectory_accessors_return_typed_errors_not_panics` | single-branch | bite: forced the `None` arm to panic instead of converting → test failed at this line specifically; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:597` | `first_way_point`'s `.front().ok_or_else` | same test | single-branch | bite: forced panic on `None`; failed at this line, isolated from `last_way_point`'s sibling; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:598` | `last_way_point`'s `.back().ok_or_else` | same test | single-branch | bite: forced panic on `None`; failed at this line, isolated from `first_way_point`'s sibling; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:696` | `way_point`'s guard, out-of-range case | `out_of_range_index_access_is_a_typed_error` | single-branch | same anchor/bite as :568 — same guard, non-empty fixture |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:697` | `way_point_mut`'s `.get_mut(index).ok_or_else` | same test | single-branch | bite: forced panic on `None` → test failed at this line; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:698` | `remove_way_point`'s explicit `if index >= len` | same test | single-branch | bite: neutralized guard → test failed at this line; reverted |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:777` | `for_group_name`'s `joint_model_group(group)?` | `unknown_group_name_is_a_typed_error_not_a_silent_whole_robot_fallback` | single-branch | bite: forced panic on the `?`'s `Err` arm → test failed; reverted |
| `crates/moveit-trajectory/tests/ruckig_smoothing.rs:203` | `trajectory.group().is_none()` — precondition sanity check on `RobotTrajectory::new`, not on `apply_smoothing`'s guard | `no_group_set_is_an_error` | not-this-family | commit `6a6b46a` (the test's substantive assertion, a few lines below this one, already checks `apply_smoothing`'s error message; this line only confirms the fixture itself has no group set) |

## moveit-planners-chomp (14)

Cost hazard note (per brief): the crate's convergence tests are slow.
Every bite below was scoped to a single `-E 'test(...)'` filter
expression, never the full crate; no bite touched a convergence test.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `cost.rs:424` | `max_quad_cost_inv_value`'s zero-dimension guard | `max_quad_cost_inv_value_rejects_zero_free_points` | single-branch | bite: neutralized guard (`if false`) → test failed (`unwrap_err()` on `Ok`); reverted |
| `cost.rs:493` | `cost()`'s length guard | `cost_and_derivative_reject_mismatched_length` | discriminating | bite: neutralized `cost()`'s guard alone (leaving `derivative()`'s intact) → test failed via nalgebra dimension panic at the `cost()` call specifically; reverted |
| `cost.rs:497` | `derivative()`'s length guard | same test | discriminating | bite (mirror): neutralized `derivative()`'s guard alone (leaving `cost()`'s intact) → test failed via nalgebra panic at the `derivative()` call specifically; reverted |
| `optimizer.rs:2933` | `add_increments_to_trajectory`'s shape guard | `add_increments_to_trajectory_rejects_shape_mismatch` | single-branch | bite: neutralized guard → test failed (nalgebra dimension panic); reverted |
| `optimizer.rs:2988` | `handle_joint_limits`'s length guard | `handle_joint_limits_rejects_joint_costs_length_mismatch` | single-branch | bite: neutralized guard → test failed (`unwrap_err()` on `Ok`); reverted |
| `crates/moveit-planners-chomp/src/planner.rs:1098` | `validate_recovery_time_limit`'s combined finite/range guard | `validate_recovery_time_limit_rejects_a_value_whose_plus_five_overflows_i32` | single-branch | bite: forced the guard's condition to `true \|\| ...` → all 4 rejection tests failed, both boundary-accept tests stayed green; reverted |
| `crates/moveit-planners-chomp/src/planner.rs:1122` | same | `validate_recovery_time_limit_rejects_a_value_whose_plus_five_underflows_i32` | single-branch | same bite |
| `crates/moveit-planners-chomp/src/planner.rs:1127` | same | `validate_recovery_time_limit_rejects_nan` | single-branch | same bite |
| `crates/moveit-planners-chomp/src/planner.rs:1132` | same | `validate_recovery_time_limit_rejects_infinity` (+inf) | single-branch | same bite |
| `crates/moveit-planners-chomp/src/planner.rs:1133` (`validate_recovery_time_limit_rejects_infinity`) | same | same test (-inf) | single-branch | same bite |
| `crates/moveit-planners-chomp/src/trajectory.rs:695` | `from_num_points`'s `num_points < 2` guard (`Error::Other`, distinct variant from the function's other guard, `Error::UnknownName` via `?`) | `from_num_points_rejects_fewer_than_two_points` | discriminating | bite: neutralized the `num_points < 2` guard → test failed (subtract-with-overflow panic downstream); the `matches!(err, Error::Other(_))` shape excludes the sibling `UnknownName` variant by construction; reverted |
| `crates/moveit-planners-chomp/src/trajectory.rs:697` | same guard, `num_points == 0` case | same test | discriminating | same bite/anchor |
| `crates/moveit-planners-chomp/src/trajectory.rs:997` | `source.group().is_none()` — precondition sanity check, not `fill_in_from_trajectory`'s own guard | `fill_in_from_trajectory_rejects_a_trajectory_with_no_group` | not-this-family | commit `77a5d7d` (the test's substantive assertion, a few lines below, already checks the error message to discriminate among `fill_in_from_trajectory`'s several `Error::other` sites; this line only confirms the fixture has no group) |
| `crates/moveit-planners-chomp/src/utils.rs:194` | `robot_state_to_array`'s two `UnknownName` sites (group vs. joint) | `robot_state_to_array_rejects_an_unknown_group_name_as_a_typed_error` | discriminating | commits `67446b2`/`a90b5dd` (checks `kind == "group"`, discriminating from the joint-name sibling) |

## moveit-planners-sbp (7 — own fence)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `goal_sampler.rs:220` | `sample_goal`'s single fallthrough `None` (budget-exhausted) | `sample_goal_carries_the_previous_draws_result_forward_as_the_next_seed` | single-branch | bite: changed the trailing `None` to `Some(...)` → test failed at this assertion; reverted |
| `goal_sampler.rs:298` | same anchor | `path_constraints_end_to_end_wired_vs_unwired` (unwired-half assertion) | single-branch | same bite/anchor as :220 |
| `nn.rs:227` | `Gnat::nearest`'s `self.root.as_ref()?` empty-index guard | `empty_index_has_no_nearest` | single-branch | bite: forced panic on the `None` arm → test failed; reverted |
| `planning_scene_validity.rs:419` | `PositionConstraint::new`'s two `UnknownName` sites (link_name vs. frame_id) | (world-object-transform test) | discriminating | commit `1397dea` (checks `kind: "frame"`/`name == "table"`; commit message records that swapping the expected kind to the link_name sibling had kept the test passing before the fix) |
| `crates/moveit-planners-sbp/src/registry.rs:1247` | `JointModelGroupSpace::new`'s single `UnknownGroup` construction site, reached through `get_planning_context` | `unknown_group_is_rejected_before_any_search_runs` | single-branch | bite: forced panic on the guard's `Err` arm → test failed; reverted. `get_planning_context`'s own doc comment states this is its only failure mode |
| `crates/moveit-planners-sbp/src/registry.rs:1867` | `PlanningContext::solve`'s single `NoGoalSample` construction site (`sample_goal(...).ok_or(...)`) | `solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable` | single-branch | bite: forced panic on the `None` arm → test failed; reverted |
| `crates/moveit-planners-sbp/src/registry.rs:2024` | `resolve_constraint_sampler`'s `None` passthrough from `moveit_constraints::select_default_sampler` | `path_constraints_solver_wiring_matches_the_call_site` | single-branch | bite: forced panic on the wrapper's `None` arm → test failed; reverted. Caveat: `select_default_sampler`'s own internal branching lives in `moveit-constraints`, outside this round's fence — not independently audited beyond confirming this test's fixture always supplies a valid `group_name`, so the sibling "unknown group" `None` inside that function is not reachable from this fixture |

## moveit-planners-stomp (7 — override, was p3-shapes')

All 7 sites carry an in-source "Assertion-discrimination sweep (round
2)" comment recording a prior sweep's reachability/isolation work in
detail. Read and independently verified against the current source
(function bodies match the comments' line-level claims); no bite
re-run since the comments already document the isolating mutation and
its outcome in both directions.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `conversion_functions.rs:329` | `set_positions`'s one panic-capable construct (`assert_eq!` length check; all other fallible steps use `?`) | `set_positions_panics_on_a_length_mismatch` | single-branch | round-report (in-source comment, round 2): `rg` for `panic!\|assert!\|assert_eq!\|unwrap(\|expect(` over the function body gives one hit — verified by direct read |
| `cost_functions.rs:582` | `sum`'s single `None` origin (`cost_fn(values)?`) | `sum_propagates_a_failing_constituent_as_none` | single-branch | round-report (in-source comment, round 2): one `?`/`None` site in the closure body — verified by direct read |
| `crates/moveit-planners-stomp/src/planner.rs:1052` | `extract_seed_trajectory`'s guard A (empty-input) of 3 `Ok(None)` sites | `extract_seed_trajectory_returns_none_for_zero_constraints` | discriminating | round-report (in-source comment, round 2/§3a): isolating mutation on A alone fails this test and leaves B's/C's tests green — verified against current source (3 `Ok(None)` sites confirmed at their stated positions) |
| `crates/moveit-planners-stomp/src/planner.rs:1139` | same function, guard C (per-joint name mismatch) | `extract_seed_trajectory_returns_none_when_joint_name_mismatches_group` | discriminating | same round-report; isolating C alone fails only this test |
| `crates/moveit-planners-stomp/src/planner.rs:1164` | same function, guard B (per-waypoint dof-count mismatch) | `extract_seed_trajectory_returns_none_when_dof_mismatches_group` | discriminating | same round-report; isolating B alone fails only this test |
| `crates/moveit-planners-stomp/src/planner.rs:1253` | `sample_goal_state`'s guard A (no sampler could be built) | `sample_goal_state_returns_none_when_no_sampler_can_be_built` | discriminating | round-report (in-source comment, round 2/§3a): isolating A alone fails this test, leaves B's test green; comment also notes B is structurally unreachable when A fires (illegal-state-unrepresentable, not just empirical) — verified against current source |
| `crates/moveit-planners-stomp/src/planner.rs:1337` | same function, guard B (sampler never converged) | `sample_goal_state_returns_none_when_the_only_candidate_sampler_never_converges` | discriminating | same round-report; isolating B alone fails only this test |

## moveit-planning (4 — override, was p3-acm's)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `pipeline.rs:779` | `generate_plan`'s single `PipelineError::NoPlanners` construction site (nullary variant) | `zero_planners_is_an_error` | single-branch | `rg` enumeration: one construction site (`pipeline.rs:432`); nullary variant carries no payload to lose, so `matches!` cannot be blinder than `==` here. Matches the census's own reconciliation note for this crate |
| `plan_responses.rs:208` | `PlanResponsesContainer` round-trip of a value pushed two lines above in the same test | `plan_responses_container_returns_pushed_outcomes_in_push_order` | not-this-family | direct read: this asserts storage/retrieval fidelity of an `Err` the test itself constructed, not selection among distinct error causes |
| `plan_responses.rs:214` | `shortest_solution`'s `best` accumulator, empty-input case | `shortest_solution_is_none_on_empty_input` | not-this-family | direct read: `best` starts `None` and the loop body never runs on an empty slice — vacuously the only possible outcome, no guard to discriminate |
| `add_time_optimal_parameterization.rs:368` | `adapt`'s single `ResponseAdapterError::Failed` construction site | `adapt_rejects_an_invalid_resample_dt_deferred_from_new` | single-branch | `rg` enumeration: one construction site (line 139) |
| `add_time_optimal_parameterization.rs:373` | the same `Failed`'s *message*, `resample_dt` guard vs every other cause routed through line 139 | `adapt_rejects_an_invalid_resample_dt_deferred_from_new` | discriminating | one construction site means `matches!(.., Failed)` at `:368` can only pin reachability; this line is what separates the two causes that reach it, and `:427` below is its negative control — a `panda_arm` with no acceleration limits reaches the same `Failed` and must NOT say `resample_dt`. Added by `973ee87` for exactly that reason |
| `add_time_optimal_parameterization.rs:422` | same single `Failed` construction site, sibling-cause path | `sibling_cause_does_not_read_as_a_resample_dt_rejection` | single-branch | same `rg` enumeration (one site, line 139): reachability of the sibling cause, not discrimination. Its purpose is to establish that the sibling cause reaches `Failed` at all, so that `:427`'s exclusion is a statement about the message and not about an unreached branch |
| `add_time_optimal_parameterization.rs:427` | the sibling cause's message, exclusionary (`!contains("resample_dt")`) | `sibling_cause_does_not_read_as_a_resample_dt_rejection` | discriminating | the half that makes `:373` mean something: without it, `contains("resample_dt")` would pass on any implementation whose `Failed` message happened to name the field on every path. Neutralising the `resample_dt` guard so its text leaks into the acceleration-limit failure reddens this line and no other |

## moveit-sampling (4 — override, was p3-shapes')

All 4 sites carry an in-source "Assertion-discrimination sweep (round
2, updated for brief section 3a)" comment with an explicit D6 check
(confirms neither in-tree caller can reach the shape guard, so this is
not a D6 finding). Read and verified against the current `new` body
(guards at the stated relative positions); no bite re-run.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `multivariate_gaussian.rs:191` | `MultivariateGaussian::new`'s guard A (shape mismatch) of 2 `None` sites | `mismatched_covariance_shape_is_none` | single-branch | round-report (in-source comment, round 2/§3a): isolating A alone fails this test and `non_square_covariance_is_none`, leaves the Cholesky-guard tests green |
| `multivariate_gaussian.rs:198` | same function, guard A (non-square) | `non_square_covariance_is_none` | single-branch | same round-report |
| `multivariate_gaussian.rs:206` | same function, guard B (`Cholesky::new(..)?`, indefinite) | `indefinite_covariance_is_none` | single-branch | round-report: isolating B alone fails this test and the zero-covariance test, leaves the shape-guard tests green |
| `multivariate_gaussian.rs:225` | same function, guard B (zero/PSD-not-PD) | `zero_covariance_is_positive_semidefinite_not_definite_and_is_none` | single-branch | same round-report |

## moveit-kinematics (3)

Evidence is this session's own round 7 (immediately prior to this
ledger), reported to and accepted by the user as-is
("The kinematics report is accepted as reported — three fresh bites,
nothing to commit").

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `cart_to_jnt.rs:666` | `search_position_ik`'s consistency-limit guard | `consistency_limit_gates_a_convergent_solution_by_distance_from_seed` | discriminating | round-report (this session, round 7): bite removed the `continue`-triggering check, flipped `None`→`Some`, isolated the solution-callback guard as unreachable in that fixture; reverted |
| `cart_to_jnt.rs:737` | `search_position_ik`'s solution-callback guard | `solution_callback_gates_acceptance_independent_of_convergence` | discriminating | round-report (this session, round 7): mirror bite, isolated from the consistency-limit guard; reverted |
| `chain.rs:488` | `ChainInfo::build`'s unknown-group-as-`UnknownName` site vs. `Error::Other` sibling | `build_reports_an_unknown_group_as_unknown_name` | discriminating | round-report (this session, round 7): relabeled to the real `Error::Other` sibling from `build_rejects_a_non_chain_group` → test failed; reverted |

## Summary

- 55/55 sites enumerated, reconciled exactly against the census's
  per-crate figures — no scanner disagreement found in any of the 7
  crates.
- No site is uncovered by any prior report; no genuinely blind
  assertion ("the finding this hunts") was found this round.
- No commits made this round — every bite run was reverted after
  confirming its result; every crate is at its pre-round tree state.
- Verdict counts: 14 discriminating, 37 single-branch, 0
  fixture-collapse-fixed, 4 not-this-family (14+37+4 = 55).

## Commands run (representative — full crate; per-guard bite commands
omitted, all followed the pattern `cargo nextest run -p <crate> -E
'test(<fn>)'`)

```
cargo fmt --all
cargo clippy -p moveit-trajectory --all-targets -- -D warnings
cargo nextest run -p moveit-trajectory
cargo clippy -p moveit-planners-sbp --all-targets -- -D warnings
cargo nextest run -p moveit-planners-sbp
cargo nextest run -p moveit-planners-chomp -E '...' (targeted, avoided convergence tests)
cargo nextest run -p moveit-planners-stomp
cargo nextest run -p moveit-planning
cargo nextest run -p moveit-sampling
```

## UNFIXED (round 8)

None. No blind assertion was found this round, so there was nothing to
fix.

The round number is in the heading because this is the only `UNFIXED`
heading in a document that goes on for a dozen more rounds, and a bare
one answers "what is open in this fence?" with "None" on behalf of
rounds it never read. Later rounds close themselves in their own
`### Result:` line instead — round 11's three fragile-but-unique
needles, round 13's `mesh_search_paths` coverage gap (closed by round
14), round 20's `Fragility flagged, not fixed`. Searching this file for
`UNFIXED` finds one section and misses all of those; searching it for
`Result:` finds them. `ledger-pilz.md` carries the same convention with
`## UNFIXED (12, now 13)`.

## Gate scope

- `moveit-trajectory`: `-p moveit-trajectory` (fmt/clippy/nextest) —
  clean, no diff to commit.
- `moveit-planners-chomp`: bites only, targeted `-E` filters per the
  brief's cost-hazard note; no full-crate nextest run this round (no
  source diff to gate).
- `moveit-planners-sbp`: `-p moveit-planners-sbp` (fmt/clippy/nextest)
  — clean, no diff to commit.
- `moveit-planners-stomp`, `moveit-planning`, `moveit-sampling`,
  `moveit-kinematics`: no source touched this round (evidence was
  round-report/commit-citation only); `cargo nextest run -p <crate>`
  confirmed green as a baseline check, no fmt/clippy re-run needed
  since nothing changed.
- Full-workspace `--workspace` variants owed before `git push` per the
  standing rule; not run this round (no commits made).

## Round 9 — family-membership rule applied (`doc/assertion-discrimination-census.md` §9)

Re-checked all 55 rows above against §9's three clauses (mechanism /
decision / subject), not just the 4 already marked `not-this-family`.

- The 4 existing `not-this-family` rows all survive: `ruckig_smoothing.rs
  :199` and `moveit-planners-chomp/trajectory.rs:997` (`fill_in_from_trajectory_rejects_a_trajectory_with_no_group`) fail clause 3 (the
  test's own `RobotTrajectory::new`/fixture-construction precondition,
  not the subject function's decision); `plan_responses.rs:208` (`plan_responses_container_returns_pushed_outcomes_in_push_order`) fails
  clauses 2 and 3 together (a container passthrough of a value the test
  pushed itself two lines above); `plan_responses.rs:214` fails clause 2
  (the comparison loop never runs on empty input — no decision to get
  wrong, unlike `nn.rs:227`'s written `self.root.as_ref()?` guard, which
  §9 cites as the contrasting in-family case). None is reclassified.
- The remaining 51 rows (`discriminating` and `single-branch`) each pass
  all three clauses: every one inspects a genuine `Err`/`None`
  fail-or-absence signal (clause 1), produced by a written guard in the
  function the test names as its subject (clauses 2 and 3). No
  corrections needed.

**In-family denominator for these 7 crates: 51 of 55** — unchanged from
this ledger's original verdict counts, because the 4 sites already
carrying `not-this-family` were the only 4 that could fail §9 in this
set, and they fail it for the same reasons their original evidence
column already gave. By crate: `moveit-trajectory` 15/16,
`moveit-planners-chomp` 12/14, `moveit-planners-sbp` 7/7,
`moveit-planners-stomp` 7/7, `moveit-planning` 2/4, `moveit-sampling`
4/4, `moveit-kinematics` 3/3.

No source or verdict cells changed this round. This section is the only
diff — gate is `cargo fmt --all -- --check` per this round's brief
(doc-only; no clippy/nextest owed).

## Round 10 — `moveit-constraints` coverage gap (override, orchestrator's own fence)

Not a census row: `assert_err_mentions`'s `assert!(rendered.contains(needle), ...)`
is a `.contains()` check, not `matches!`/bare `.is_err()`/`.is_none()`, so
this site is outside census §1's syntactic scan and adds nothing to the
289. It is the same defect family the sweep exists to catch — a guard an
assertion could be blind to — found as a coverage gap rather than an
existing site's misclassification.

`PositionConstraint::new`'s first fallible call, `model.link_model
(link_name)?` (`crates/moveit-constraints/src/position.rs:165`), is a sibling of `resolve_frame`'s
frame-id guard (`crates/moveit-constraints/src/position.rs:108-109`, called from `:188`): both reach
`Error::UnknownName`, differing only in `kind` (`"link"` vs `"frame"`).
Every one of this crate's own `PositionConstraint::new` test call sites
used a valid `link_name`; only the frame-id branch had a rejection test
(`new_rejects_unresolvable_mobile_frame`, `decide.rs:549`) before commit
`2201d35` (already on `main`, merged into this branch via `git merge
--ff-only main` before this round's work started — not written by this
ledger's author this round) added `new_rejects_unknown_link`
(`decide.rs:530`).

The commit's own message claims a mutation-confirmed bite. Independently
re-run this round, not relayed:

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `moveit-constraints/src/position.rs:165` | `PositionConstraint::new`'s `link_model(link_name)?` guard, sibling of `resolve_frame`'s frame-id guard at `:108-109` | `new_rejects_unknown_link` | discriminating | bite (this round): `model.link_model(link_name)?` replaced with `.unwrap_or(&model.link_models()[0])` — `new_rejects_unknown_link` FAILED, `new_rejects_unresolvable_mobile_frame` stayed GREEN; reverted. Mirror: `resolve_frame`'s `if !model.has_link_model(..) && ..` gated `if false && ...` — `new_rejects_unresolvable_mobile_frame` FAILED, `new_rejects_unknown_link` stayed GREEN; reverted. `git status --short` clean before and after. |

**Call-site count, re-derived fresh (not relayed from `2201d35`'s own
message):** the commit's "23" is already stale — `main` has moved since
`2201d35` landed. Live count via `rg -n 'PositionConstraint::new\(' <file>`
across the same four test files: `decide.rs` 12, `utils_parity.rs` 2,
`constraint_sampler_manager.rs` 6, `ik_sampler.rs` 13 = **33 call sites**,
one more crate (`sampler_self_validation.rs`) has none. Of those 33, one
(`decide.rs:535`, the body of `new_rejects_unknown_link` itself) is the
*deliberately* invalid case; `rg -n '"no_such_link"|no_such_link'` finds no
other hit. Spot-read two of the remaining 32 rather than trust the count:
`decide.rs:258` (`satisfied_when_link_origin_is_inside_the_region`)
passes `"panda_link8"` and `.unwrap()`s the result;
`crates/moveit-constraints/tests/ik_sampler.rs:75` (`sampling_volume_sums_sphere_region_bodies`) also
passes `"panda_link8"`. Both are real links on the `panda_model()`
fixture, both call sites `.unwrap()` (would panic, not silently pass, if
the link lookup failed) — the "all other 32 use a valid `link_name`" claim
holds on direct read, not just by the string-search's own count.

**Message-text-fragility caveat.** `new_rejects_unknown_link` and
`new_rejects_unresolvable_mobile_frame` both go through
`assert_err_mentions`, a substring match on the rendered `Display` text —
by itself, a test that distinguishes two sibling branches only by message
wording is only as strong as that wording staying put. That is not the
only evidence here, though: the bite tabled above mutates the *guard
code*, not the message, and it discriminates at that level — neutralizing
`link_model(link_name)?` fails `new_rejects_unknown_link` regardless of
what `Error::UnknownName`'s `Display` impl renders, because the mutation
changes which branch runs, not what it prints. If someone changed `"no
link named {name}"` to different wording without touching either guard,
the two tests would not silently lose their power to discriminate; they
would fail loudly (the needle no longer matches), which is the safe
failure mode — caught by CI, not a silent pass-through. The message check
is a second, stricter layer on top of a discrimination proof that does
not itself depend on the message text.

No source changes this round — the test already existed on `main`. One
commit this round: this ledger row.

## Commands run (round 10)

- `cargo nextest run -p moveit-constraints -E 'test(new_rejects_unknown_link) or test(new_rejects_unresolvable_mobile_frame)'` — baseline 2/2 pass, then run again after each mutation (1 fail/1 pass both times, as tabled above), then again after each revert (2/2 pass)
- `cargo fmt --all -- --check` — clean
- `cargo clippy -p moveit-constraints --all-targets -- -D warnings` — clean
- `cargo nextest run -p moveit-constraints` — 100/100 pass

Gate scope: `-p moveit-constraints` (fmt + clippy --all-targets -D
warnings + nextest), per this round's brief. No workspace-wide pass owed
this round.

## Round 11 — the `.contains()`-shaped message-substring family, `crates/`

Fence this round: `crates/` minus `moveit-planners-pilz` (p1-joints' this
round) and `ros/` (p9-ros's, always). This section covers `assert_err_mentions`
and every bare `assert!(... .contains(...) ...)` whose asserted value is a
rendered error/diagnostic message — the shape census §1's grammar
(`matches!`/bare `.is_err()`/`.is_none()`) does not scan for at all, so
these rows add nothing to the 289 and are not a re-classification of any
existing row.

### Count: reconciling 43 vs. the committed instrument's 70 vs. this round's 82

Three numbers, three instruments, over the same `crates/` tree (all
crates, pilz included — the fence narrowing only affects which rows I
touch, not the honest total):

- **43** — the brief's own figure. No methodology was given with it, and
  none survived: it is roughly half of both other counts and I found no
  subset of the true population that lands there.
- **70** — `tools/ci/count-coarse-assertions.py`'s `contains_msg` count
  for `crates/` (`python3 tools/ci/count-coarse-assertions.py crates`,
  re-run this round). Its own doc comment names one blind spot: it decides
  `contains_msg` vs `contains_member` by looking 60 bytes back from
  `.contains(` for a rendering call, so a helper that renders on one line
  and asserts on a later one reads as `contains_member` — `assert_err_mentions`
  is exactly that shape (`crates/moveit-constraints/tests/decide.rs:79-86`
  defines it, `:83` is the one `assert!` inside it, which the script does
  count once, as `contains_msg`, since the binding is literally named
  `rendered`). Its **8** call sites in `crates/` (`decide.rs:362, 380,
  438, 508, 534, 553, 742, 781` — `rg -n 'assert_err_mentions\(' crates/`)
  are plain function calls with no `.contains(` token in them at all, so
  the script cannot see them as sites; net effect, +7 relative to the
  script's one combined hit (8 calls, minus the 1 definition-hit already
  counted).
- A **second, undocumented blind spot**, found this round: the same
  60-bytes-back heuristic also misses a message check when the rendered
  text lives in a `detail: Option<String>` *field* read through a closure
  parameter rather than a `rendered`/`message`/`msg`-named binding or a
  `to_string()`/`unwrap_err()` call directly at the `.contains(` site.
  Five sites take this shape: `crates/moveit-model/src/robot_model.rs:2472,
  2512, 2542` (`detail.as_deref().is_some_and(|d| d.contains(...))` —
  `d` carries no rendering-call token in the 60 bytes back) and
  `crates/moveit-planning/src/request_adapters/check_start_state_collision.rs:161,
  162` (`detail.contains(...)` after a `match err { StartStateInCollision
  { detail, .. } => ... }`, so `detail` alone is in the window). All five
  are genuine message-substring checks — `robot_model.rs`'s own doc
  comment on the first of the four sibling `UnsupportedLinkGeometry` tests
  says so directly: "Distinct from the three tests around it by the
  `detail` phrase: all four produce `kind == "mesh"`, so matching only the
  variant would not say which branch fired." The script classifies all
  five as `contains_member`; +5 relative to its count.

  70 (script) + 7 (`assert_err_mentions` calls, undercounted) + 5 (`detail`-field
  checks, undercounted) = **82**, which is exactly this round's independent
  count, arrived at before running the script at all (own scanner:
  `scan_contains2.py`, a paren-depth/comment-masking `assert!(` walker
  matching the census's own §1 method, widened to flag any body containing
  `.contains(` — 137 raw hits over `crates/`, hand-classified against
  false-positive categories: bitflags `.contains(Action::X)` (25),
  numeric-range `.contains(&v)` (28), `Vec`/`HashSet`/`HashMap`
  membership `.contains(&x)`/`.contains_key(...)` (14), Debug/Display-of-a-
  *success*-value `.contains(...)` (12, `robot_trajectory.rs`'s printed-output
  checks and `debug.contains("dirty: None")`), leaving 74 bare-shape hits
  plus the 8 `assert_err_mentions` calls the bare-body scan cannot see at
  all = 82; false-positive/true-positive split cross-checked by re-summing
  both halves against the 137 raw total, both routes agree).

  **82 is the right number for all of `crates/`.** 70 undercounts by a
  documented mechanism plus one more this round found and is reporting so
  it can be fixed at the instrument, not just noted here. 43 has no
  surviving methodology.

This round's own fence (pilz excluded) covers **77** of the 82:
`moveit-planners-pilz/src/path_circle.rs:558, 590` and
`trajectory_generator_ptp.rs:449, 466, 495` (5 sites) are p1-joints'
this round and are not analyzed below.

### Per-site verdicts (77 sites, `crates/` minus pilz and ros/)

Method, per the brief: read the needle, enumerate every *other* message
the same subject function can produce (by reading its source, not
guessing), check substring containment. `discriminating` = needle is not
a substring of any sibling message, sibling list given. A verdict of
`discriminating` below that also cites an existing in-code comment means
this round found and transcribed a prior round's own bite evidence
(mostly "Assertion-discrimination sweep (round 2)" comments already
present in these files) rather than re-deriving it from nothing; verdicts
with no such citation are this round's own reading.

**`moveit-constraints/tests/decide.rs`** — the one `assert_err_mentions`
definition in `crates/` (`:79-86`); `rg -n 'fn assert_err' -e 'fn.*mentions'
crates/` and `rg -n 'assert_err_mentions' crates/` both confirm no second
copy exists anywhere in this fence. The brief's "planning.rs:331,
trajectory.rs:248" per-file-duplication citation does not resolve inside
`crates/` at all: `crates/moveit-trajectory/src/trajectory.rs:248` is
`Trajectory::acceleration`'s body, not a test helper, and no file named
literally `planning.rs` exists in `crates/`; both names *do* exist at
`ros/moveit-ros/src/planning.rs` and `ros/moveit-ros/src/trajectory.rs` —
p9-ros's fence, not read further here. Within `crates/`, `assert_err_mentions`
is defined once and not duplicated.

`PositionConstraint::new` (`crates/moveit-constraints/src/position.rs:156-197`) siblings: `link_model(link_name)?`
→ `no link named "X"`; `frame_id.trim().is_empty()` → `no frame specified
for position constraint`; `shapes.is_empty()` → `PositionConstraint needs
at least one constraint region`; `Body::from_shape` → `Ok(None)` →
`shape {shape:?} has no bodies:: counterpart to build a constraint region
from`; `Body::from_shape` → `Err` → propagates `bodies.rs`'s own messages
(`convex mesh body requires at least one vertex` / `convex hull
computation failed: {e}`); `resolve_frame` → `no frame named "X"` (kind
`frame`).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `decide.rs:362` | `no frame specified for position constraint` | discriminating | none of the other 5 messages contain it — line corrected round 17 (was line 372, the message-string argument line inside this multi-line `assert_err_mentions` call, not the call's own opening line the scanner cites) |
| `decide.rs:380` | `needs at least one constraint region` | discriminating | substring only of the shapes-empty message; not in the other 5 — line corrected round 17, was line 390 |
| `decide.rs:438` | `has no bodies:: counterpart` | discriminating | substring only of the `Ok(None)` message; not in the other 5 — line corrected round 17, was line 448 |
| `decide.rs:508` (`new_rejects_a_mesh_whose_body_construction_fails`) | `convex mesh body requires at least one vertex` | discriminating | exact text of `bodies.rs:2542-2543`'s guard; not in the other 5, nor in `bodies.rs`'s own sibling `convex hull computation failed: {e}` — line corrected round 17, was line 518 |
| `decide.rs:534` | `no link named "no_such_link"` | discriminating | not in the `frame`-kind message (different `kind`) nor the other 4 — line corrected round 17, was line 544 |
| `decide.rs:553` | `no frame named "no_such_frame"` | discriminating | not in the `link`-kind message nor the other 4 — line corrected round 17, was line 563 |

`OrientationConstraint::new` (`crates/moveit-constraints/src/orientation.rs:195-248`) has only two
fallible sites: `link_model(link_name)?` → `no link named "X"`;
`!model.has_link_model(frame_id) && frame_id != model.model_frame()` →
`no frame named "X"` (one guard, reached both by an unresolvable name and
by the empty string — no separate empty-`frame_id` branch exists for this
type, unlike `PositionConstraint`; the test doc comment at `:770-780`
says so directly).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `decide.rs:742` | `no frame named "no_such_frame"` | discriminating | not in the `link`-kind message; the other same-guard instance (`decide.rs:781`) differs by quoted content only — line corrected round 17, was line 756 |
| `decide.rs:781` | `no frame named ""` | discriminating | same as above, reversed; deliberately the same guard as `:742`, not a collision — the doc comment says so — line corrected round 17, was line 795 |

**`moveit-constraints/tests/sampler.rs`** — subject `JointConstraintSampler::new`
(`crates/moveit-constraints/src/sampler.rs:179-259`), three sites: unknown group → `Error::UnknownName`;
empty intersection → `"JointConstraintSampler: no possible values for
joint variable '{}': min_bound {} > max_bound {}"`; no applicable
constraint → `"JointConstraintSampler: no joint constraints apply to
group '{group_name}'"`.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `crates/moveit-constraints/tests/sampler.rs:84` | `panda_joint1` | discriminating | not in the group-name message for this call (`group_name = "panda_arm"`) |
| `crates/moveit-constraints/tests/sampler.rs:221` | `panda_arm` | discriminating, **fragile** | for this call's inputs, unique; flagged because `panda_arm` is a prefix of `panda_arm_hand`, so if this needle were ever reused against an unknown-group failure for `"panda_arm_hand"` it would spuriously match — not fixed, no such reuse exists today |

**`moveit-distance-field/src/voxel_grid.rs`** — already bite-checked
in-code (`:439-449`, `:479-487`, `:494-507`); this round transcribes,
does not re-derive. `GridGeometry::new`'s three guards: `resolution` not
finite/positive → `... must be finite and positive`; `size.x`/`size.y`/`size.z`
over-resolution overflow, one message per axis naming that axis.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `voxel_grid.rs:454` | `must be finite and positive` | discriminating | in-code bite: disabling the resolution guard alone leaves this assertion red (the overflow guard's message never contains this phrase) |
| `voxel_grid.rs:456` (`rejects_non_positive_resolution`) | `must be finite and positive` | discriminating | same guard, negative-resolution boundary; same bite |
| `voxel_grid.rs:512` | `size.y` | discriminating | in-code bite: isolates the y-axis overflow guard from x/z's, confirmed by disabling y's guard alone |

**`moveit-geometry/src/bodies.rs`** — already bite-checked in-code
(round 2 comments at each site cited below).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `bodies.rs:3953` | `requires at least one vertex` | discriminating | `build_mesh_data`'s two `Error::Construct` sites; bite-checked (`:3937-3948`) |
| `bodies.rs:4055` | `radius` | discriminating | `Cylinder::recompute`'s two sequential guards; bite-checked, message-swap confirmed (`:4045-4051`) |
| `bodies.rs:4066` | `length` | discriminating | same pair, reverse guard |
| `bodies.rs:4125` | `radius` | discriminating | `Cylinder::set_padding`'s two guards via `recompute`; bite-checked, message-swap confirmed (`:4106-4118`) |

**`moveit-kinematics/src/chain.rs`** + **`moveit-kinematics/tests/ik_fk_roundtrip.rs`**
— subject `ChainInfo::build` (`chain.rs:145-294` (`build`)), four Err messages:
`not a chain`, `{} DOF; only single-DOF joints are supported`,
`unsupported type {}`, `not itself in the group`. `NewtonRaphsonSolver::new`
(`newton_raphson.rs:83-109`) only forwards `ChainInfo::build`'s error —
verified by reading the function, no extra branch.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `chain.rs:469` (`build_rejects_a_non_chain_group`) | `not a chain` | discriminating | not in the DOF, unsupported-type, or mimic-master messages |
| `chain.rs:512` (`build_rejects_a_multi_dof_joint`) | `DOF` | discriminating | not in the other 3 |
| `chain.rs:558` | `not itself in the group` | discriminating | not in the other 3 |
| `ik_fk_roundtrip.rs:281` | `not a chain` | discriminating | `NewtonRaphsonSolver::new` forwards the identical 4-message surface; same check as `chain.rs:469` |

**`moveit-model/src/robot_model.rs`** — two independent guard groups.
`RobotModel::from_urdf_and_srdf`'s root-link resolution (`:215-230`): `[]`
→ `has no root link`; `names` (>1) → `has {} root links, expected exactly
one`. `mesh_collision_*` diagnostics (`:2380-2555`ish, four
`Diagnostic::UnsupportedLinkGeometry` producers sharing `kind == "mesh"`,
distinguished only by `detail`): unresolved path, non-STL file, unreadable
file, malformed STL bytes.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `robot_model.rs:2328` | `has no root link` | discriminating | not in the multi-root message |
| `robot_model.rs:2343` (`multiple_root_links_errors`) | `root links, expected exactly one` | discriminating | not in the no-root message |
| `robot_model.rs:2513` (`mesh_collision_resolving_to_a_non_stl_file_is_skipped_with_a_diagnostic`) | `only STL is supported` | discriminating | not in "it could not be read" / "failed to parse" / the 4th (equality-checked, not `.contains()`) sibling; doc comment names all four explicitly |
| `robot_model.rs:2553` (`mesh_collision_resolving_to_an_unreadable_file_is_skipped_with_a_diagnostic`) | `it could not be read` | discriminating | not in the other 3 |
| `robot_model.rs:2583` | `failed to parse` | discriminating | not in the other 3 |
| `robot_model.rs:2676` | `Box dimensions` | discriminating, **breadth caveat** | subject is `build()`, the whole model pipeline, not a narrow constructor — sibling surface not exhaustively enumerable the way a single function's is; the phrase itself (from `bodies.rs:2210`) is distinctive and not found reused elsewhere in this crate |

**`moveit-planners-chomp/src/cost.rs`** (`ChompCost::new`, three
`Error::other` sites, in-code bite noted at `:384-391`) + **`optimizer.rs`**
(`calculate_smoothness_increments` two sites at `:2371-2379`;
`calculate_total_increments` three sites at `:2431-2440`) +
**`trajectory.rs`** (`ChompTrajectory::from_duration` three sites,
in-code bite at `:701-711`; `assign_chomp_trajectory_point_from_robot_state`
two sites at `:912-919`; `fill_in_from_trajectory` several sites at
`:952-960`) — all already documented in-code with the exact reasoning
this round would otherwise derive.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `cost.rs:391` | `DIFF_RULES rows` | discriminating | not in "DIFF_RULE_LENGTH-1" or "singular" |
| `cost.rs:404` | `DIFF_RULE_LENGTH-1` | discriminating | not in the other 2 |
| `cost.rs:436` | `singular` | discriminating | not in the other 2 |
| `optimizer.rs:2828` | `joint_costs has` | discriminating | not in `ChompCost::derivative`'s joint_trajectory-length message |
| `optimizer.rs:2897` (`calculate_total_increments_rejects_column_count_mismatch`) | `columns` | discriminating | in-code: "appears only in this one's message" among 3 sites |
| `crates/moveit-planners-chomp/src/trajectory.rs:712` | `discretization must be finite and positive` | discriminating | not in "would require more than" or `from_num_points`'s num_points<2 message |
| `crates/moveit-planners-chomp/src/trajectory.rs:727` (`from_duration_rejects_negative_discretization`) | same | discriminating | same guard, negative-discretization boundary |
| `crates/moveit-planners-chomp/src/trajectory.rs:748` | same | discriminating | same guard, negative/negative boundary that divides positive |
| `crates/moveit-planners-chomp/src/trajectory.rs:766` | `would require more than` | discriminating | not in the discretization message |
| `crates/moveit-planners-chomp/src/trajectory.rs:925` | `active joints, but this ChompTrajectory has` | discriminating | not in the multi-DOF-joint message |
| `crates/moveit-planners-chomp/src/trajectory.rs:968` | `variables; ChompTrajectory requires every active joint` | discriminating | not in the column-count message |
| `crates/moveit-planners-chomp/src/trajectory.rs:1004` | `requires trajectory.group() to be Some` | discriminating | not in any of `fill_in_from_trajectory`'s several other reachable messages |

**`moveit-planners-stomp/src/filter_functions.rs`** — `enforce_position_bounds`
has exactly one Err site (`require_single_variable`'s guard, one call per
joint in a loop, "no sibling branch" per the in-code comment at
`:288-306`); the compound needle names the joint and its variable count
within that one message, not two different guards.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `filter_functions.rs:314` (`enforce_position_bounds_rejects_a_multi_variable_joint`) | `world_joint` AND `3 variables` | discriminating | single reachable guard; compound needle verifies message content, not branch identity — no sibling to collide with |

**`moveit-planning/src/request_adapters/check_start_state_collision.rs`**
— `RequestAdapterError` has exactly two variants (`crates/moveit-planning/src/error.rs:22-42`):
`StartStateInvalid` (no `detail` field) and `StartStateInCollision {
detail, .. }`. The test's `match err { StartStateInCollision { detail, ..
} => ... }` already pins the variant before either `.contains()` call
runs; the checks then verify *which* collision is reported.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `check_start_state_collision.rs:161` | `contact(s) detected` | discriminating | variant already pinned by the match; `StartStateInvalid` carries no `detail` to collide with |
| `check_start_state_collision.rs:162` | `engulfing_box` | discriminating | same; names the specific shape this test added, not reusable text from any fixed message |

**`moveit-smoothing/src/butterworth.rs`** — `ButterworthFilter::new`'s
four `Error::Construct` sites, in-code bite-checked at each site
(`:150-200`).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `butterworth.rs:153` | `unstable` | discriminating | message-swap bite-checked against the other 3 |
| `butterworth.rs:162` | `scale_term_` | discriminating | not in "resulted in feedback term of 0" or "feedback_term_" (trailing underscore differs from the prose phrase) |
| `butterworth.rs:172` | `resulted in feedback term of 0` | discriminating | not in the other 3 |
| `butterworth.rs:183` | same | discriminating | same guard, adjacent boundary (`coeff = 1 + 1e-10`) |
| `butterworth.rs:200` | `feedback_term_` | discriminating | not in "resulted in feedback term of 0" (no trailing underscore there) or the other 2 |

**`moveit-smoothing/src/acceleration_filter.rs`** — `joint_acceleration_bounds`
(two `Error::Other` sites, bite-checked at `:446-454`) and
`AccelerationLimitedFilter::do_smoothing`/`reset` (bite-checked at
`:520-524`, `:538-541`).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `acceleration_filter.rs:466` | `must have acceleration joint limits` | discriminating | message-swap bite-checked against the sibling single-DOF-active-joint guard |
| `acceleration_filter.rs:525` (`multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins`) | `planar_joint` AND `3` | discriminating | `message = err.to_string()` (script misses the `err_shape` heuristic here only because the binding is on a prior line, not because it isn't one) |
| `acceleration_filter.rs:542` | `Make sure the reset was called` | discriminating | message-swap bite-checked against the sibling length-mismatch guard |

**`moveit-smoothing/src/ruckig_filter.rs`** — mirror structure to
`acceleration_filter.rs`, same bite-checked pattern.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `ruckig_filter.rs:388` | `acceleration limit defined` | discriminating | message-swap bite-checked against the sibling single-DOF/velocity/jerk guards |
| `ruckig_filter.rs:530` | `planar_joint` AND `3` | discriminating | same shape as `acceleration_filter.rs:525` |
| `ruckig_filter.rs:613` | `must each have length` | discriminating | Renumbered twice (`:539` → `:604` → `:613`; the last shift is Round 6's `:289` comment fix, +9). The "message-swap bite-checked against the sibling ruckig-update-failure site" this row used to claim was never possible: that sibling (`:289`) is unreachable under `Ruckig<0, IgnoreErrorHandler>`, whose handlers return `Ok(())` unconditionally, so `update()` cannot return `Err` — see p1-fixtures' Round 6 section. The verdict survives on the needle instead: `do_smoothing`'s own guard (`:268`) is the only reachable producer on this call path. |

**`moveit-srdf/tests/boundaries.rs`** — `SrdfModel::parse_str`'s two
`Error::Parse` sites, already discriminated by p1-joints this session
(`83f8ea0`); this round re-verifies rather than re-fixes (user's standing
instruction: no further edits to work another panel's round already
closed).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `crates/moveit-srdf/tests/boundaries.rs:49` | `opened but never closed` | discriminating | roxmltree's own `UnclosedRootNode` message (verified against the vendored `roxmltree-0.21.1/src/parse.rs:254-255`: `"the root node was opened but never closed"`, generic, names no element) — not in the other guard's message |
| `crates/moveit-srdf/tests/boundaries.rs:55` | `robot`, **fragile** | discriminating | not in roxmltree's message (verified: that message never names an element at all) today; flagged because `robot` is the XML root tag's own literal name and a future third `Error::Parse` site that echoes a tag name could collide — not fixed, no such site exists |

**`moveit-state/tests/jacobian.rs`** — subject `Posed::jacobian`
(`crates/moveit-state/src/state.rs:1275-1364`ish), three messages: `is_chain()` false → `the
group '{}' is not a chain; cannot compute Jacobian`; tip-not-descendant →
`link '{}' does not belong to the chain rooted by group '{}'`; unsupported
per-joint-kind dispatch → `joint '{}' has unsupported type {} for
Jacobian computation`. Not previously bite-annotated in-code; this round's
own reading.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `jacobian.rs:140` | `not a chain` | discriminating | not in the link-not-in-chain or unsupported-type messages; `panda`'s `hand` group hits this guard via a one-root-fails-adjacency `is_chain() == false` |
| `jacobian.rs:159` | `not a chain` | discriminating | same guard, reached instead via `pr2`'s `arms` group having two roots — deliberately the same message for two different `is_chain()`-false causes, not a collision (`JointModelGroup::is_chain` collapses both into one boolean by design) |
| `jacobian.rs:192` | `unsupported type` | discriminating | not in the other 2 |

**`moveit-trajectory/src/path.rs`** — `Path::create`'s three
`Error::Construct` sites, in-code noted at `:214-216`.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `path.rs:217` | `at least 2 waypoints` | discriminating | not in the max_deviation or 180-deg message |
| `path.rs:223` | same | discriminating | same guard, 0-waypoint boundary |
| `path.rs:236` | `max_deviation must be greater than 0.0` | discriminating | not in the other 2 |
| `path.rs:242` | same | discriminating | same guard, negative boundary |
| `path.rs:318` (`a_180_degree_turn_is_rejected`) | `180 deg` | discriminating | not in the other 2 |

**`moveit-trajectory/src/time_optimal_trajectory_generation.rs`** — three
independent guard groups.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `time_optimal_trajectory_generation.rs:1086` | `exceeding the` | discriminating at the guard, fixture-pinned at the operand | One **producer** (`:855`), still globally unique — `rg -n 'exceeding the' <file>` gives `:855` plus four `.contains` call sites (`:1086,:1113,:1179,:1569`; re-derived this round, was `:1081,:1108,:1174,:1564` before a merge shifted the file). Round 17's caveat read the 4th call site as a 4th producer and called the premise broken; it is not. What the needle genuinely cannot do is name which *operand* of the folded guard fired — `!raw_sample_count.is_finite() \|\| raw_sample_count > MAX_RESAMPLE_SAMPLE_COUNT` is one `Err` site over two branches. That is pinned by the fixtures instead: a NaN duration cannot satisfy `>`, so `:1569` (`b12b358`) reaches the finiteness operand and the other three reach the bound. Cross-referenced by p9-ros's row for the same four sites. |
| `:1113` | same | discriminating | same guard, subnormal-`resample_dt` boundary |
| `:1179` | same | discriminating | same guard, `usize::MAX`-targeting boundary |
| `:1435` | `4` AND `7`, **fragile** | discriminating | the mimic-dimension-mismatch guard (`:770-780`) is the *only* reachable `Err` once `max_velocity.len() != group.variable_names().len()`, which is unconditional for any group with a mimic joint — no sibling branch is reachable to collide with today, but the needles themselves are bare digits, the weakest form found this round; any future guard added ahead of this one in `do_time_parameterization_calculations` that also renders a floating-point value ending in 4 or 7 would defeat it. Not fixed — not currently blind, and the brief says do not fix speculatively. |
| `:1496` | NOT `invalid max_acceleration`, exclusionary | discriminating | a custom (caller-supplied) acceleration limit skips the bounds-fallback validation entirely (`:592-606`); the negation proves the failure did not come from that branch, which is what the test claims — verified the custom-limit code path truly has no validation call on it |
| `:1596` | `num_waypoints > 1` | discriminating | `totg_compute_time_stamps`'s only guard before delegating to `compute_time_stamps`; re-derived this round, was `:1511` before a merge shifted the file onto an unrelated test (`has_mixed_joint_types_boundary`) |
| `:1602` | same | discriminating | same guard, `num_waypoints = 0` boundary; re-derived this round, was `:1517` |

**`moveit-trajectory/src/trajectory.rs`** — `Trajectory::create`'s three
`Error::construct` sites (`:121-176`), already self-documented in-code
(`:1427-1433`, the `DISTINGUISHING_PHRASE` constant and its own comment).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `crates/moveit-trajectory/src/trajectory.rs:1434` | `after integrateForward and integrateBackward` | discriminating | not in "the time step is <= 0.0" or "after the second integrateBackward pass" (the word order differs from this needle) |
| `:1440` | same | discriminating | same guard, second velocity-vector case |
| `:1446` | same | discriminating | same guard, third case |
| `:1473` | `the time step is <= 0.0` | discriminating | not in the other 2 |

**`moveit-trajectory/tests/robot_trajectory.rs`** — `RobotTrajectory::insert_way_point`'s
two `Error::other` sites, in-code bite noted at `:502-504`.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `crates/moveit-trajectory/tests/robot_trajectory.rs:542` (`insert_way_point_at_zero_rejects_a_nonzero_dt`) | `duration_from_previous[0] must be 0.0` | discriminating | not in `index_error`'s message |
| `crates/moveit-trajectory/tests/robot_trajectory.rs:704` | `out of bounds` | discriminating | exact substring of `index_error`'s message only; not in `first_duration_error`'s or `empty_error`'s |

**`moveit-trajectory/tests/ruckig_smoothing.rs`** — `apply_smoothing`'s
three `Error::other` sites, in-code noted at `:204-207` (cited `:198-201`
before drift).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `crates/moveit-trajectory/tests/ruckig_smoothing.rs:208` | `did not set the group` | discriminating | not in "ruckig calculate failed: {error}" or the third (smoothing-result-failure) message |

### Result: 0 blind sites, 3 fragile-but-currently-unique needles flagged, not fixed

All 77 in-scope sites verdict `discriminating`. No needle collided with a
sibling message under this round's reading. Three fragility notes are
recorded above (`crates/moveit-constraints/tests/sampler.rs:221`, `crates/moveit-srdf/tests/boundaries.rs:55`,
`time_optimal_trajectory_generation.rs:1435`) per the brief's instruction
to flag rather than speculatively fix a needle that is unique today but
could collide if a sibling message is reworded. No fixes, no commits for
findings this round — nothing in the 77 met the blind-site bar. One
commit this round: this ledger section, plus a separate report to
`tools/ci/count-coarse-assertions.py`'s owner naming the second
`detail`-field blind spot found while reconciling the count (not a fix
made here — that script is not this fence's to edit).

## Commands run (round 11)

- `git merge --ff-only main` — picked up `tools/ci/count-coarse-assertions.py` (`6a14a89`) and `crates/moveit-model/src/robot_model.rs`'s p1-fixtures update
- `python3 tools/ci/count-coarse-assertions.py crates` — 70 `contains_msg` hits, cross-checked against this round's own scanner
- `rg -n 'assert_err_mentions' crates/` — 1 definition (`decide.rs:79`), 8 call sites, all in `decide.rs`; no per-file duplication within this fence
- `rg -n 'exceeding' crates/moveit-trajectory/src/time_optimal_trajectory_generation.rs` — 1 occurrence, confirms `:1070/1097/1163`'s needle is globally unique in-file, not just locally. **Round-11 tree, past tense**: those three line numbers are long dead and the file now holds one producer (`:855`) plus four `.contains` call sites. The uniqueness this line records is the *producer's*, and that still holds — see the corrected row above, which also names what the needle cannot do (say which operand of the folded `is_finite`/bound guard fired)
- `cargo fmt --all -- --check` — clean
- No source changes this round (0 blind sites), so no `-p <crate>` clippy/nextest gate was owed beyond the doc-only fmt check above.

Gate scope: doc-only round, `cargo fmt --all -- --check`. No crate's
clippy/nextest gate owed — no source file in this fence changed.
beyond the standing pre-push rule.

## Round 12 — reconciling 77 against the corrected tool's 81, `crates/`

`git merge main` (merge commit `63e043a`) pulled in `ccac7ea`
("count an assertion helper's call sites, not its body"), which fixed a
structural gap p9-ros found in `ros/`: an `assert!` inside a helper fn
that asserts on its own parameter is a mechanism, not a site — its
sites are the helper's call sites. The fixed tool now emits a helper's
own body as scope `helper_body` ("exclude it from any count", per the
script's own header) and each call site as kind `via:<fn>`. Re-run:
`python3 tools/ci/count-coarse-assertions.py crates | rg -v
'moveit-planners-pilz'`. Reported for this fence: **81** (`contains_msg`
plus `via:` line counts summed without excluding overlap).

**81 does not survive a re-derivation.** Kind breakdown of the filtered
output:

- `contains_msg`, any scope: **65** lines — but one of those 65 is
  `decide.rs:83`, `assert_err_mentions`'s own body, scope `helper_body`,
  which the tool's own docstring says to exclude. Non-`helper_body`
  `contains_msg`: **64**.
- `via:` lines: **12**, but only **8** are `via:assert_err_mentions`
  (`decide.rs:362, 380, 438, 508, 534, 553, 742, 781` — exact match
  against round 11's table, byte for byte). The other 4 are
  `via:check_scenario` (`crates/moveit-distance-field/tests/oracle_parity.rs:296,304`)
  and `via:rows_to_string` (`crates/moveit-stomp-core/src/utils.rs:641,654`).
  Read their helper bodies (`oracle_parity.rs:264,279`,
  `crates/moveit-stomp-core/src/utils.rs:499`): `check_scenario` asserts `.is_none()`/`.is_some()`
  on `actual.voxel`, `rows_to_string` asserts `!rows.is_empty()`.
  Neither renders or checks an error *message* — they are
  Option-presence and collection-emptiness checks, a different
  assertion family from this round's ("`assert_err_mentions`-style and
  bare `.to_string().contains(...)`" — message-substring
  discrimination only). Their 4 call sites are not this fence's family.

  65 + 12 = 77 with no double count between those two buckets (`kind`
  is either exactly `contains_msg` or starts with `via:`, never both).
  The reported 81 is 77 plus the 4 `helper_body` lines (`decide.rs:83`,
  `oracle_parity.rs:264,279`, `crates/moveit-stomp-core/src/utils.rs:499`) added back on top —
  i.e., counted once as the kind they carry and a second time as
  `helper_body`, exactly the double-count the tool's own comment warns
  against ("exclude it from any count").

**The corrected tool still cannot see the second blind spot this round
already documented.** Re-checked this round: `robot_model.rs:2472,
2512, 2542` and `check_start_state_collision.rs:161,162` — all five
still classify as `contains_member`, not `contains_msg`, in the
`ccac7ea` output. That heuristic (60 bytes back for a rendering token)
is untouched by the helper-body/`via:` fix, which addresses a different
gap (cross-statement helper calls, not closure-bound field reads); the
two blind spots are orthogonal and neither fix subsumes the other.

**True count for this fence: 64 (direct `contains_msg`, non-`helper_body`)
+ 5 (`detail`-field sites still misclassified as `contains_member`) + 8
(`via:assert_err_mentions`, the one helper in this family) = 77** — the
same 77 already swept and verdicted in round 11, not a new population.
No new call sites, no new blind-spot-adjacent sites, nothing to
re-verdict.

**Spot-read, four sites from four different crates**, cross-checked
line-for-line against round 11's own tables (not re-derived from
scratch — confirming the tool's classification agrees with source
already read):

| file:line | tool's kind | round 11 verdict | match |
|---|---|---|---|
| `moveit-distance-field/src/voxel_grid.rs:454,456,512` | `contains_msg` | discriminating (3/3) | line numbers and needles identical |
| `moveit-geometry/src/bodies.rs:3953,4055,4066,4125` | `contains_msg` | discriminating (4/4) | line numbers and needles identical |
| `moveit-planners-chomp/src/{cost.rs,optimizer.rs,trajectory.rs}` (12 sites) | `contains_msg` | discriminating (12/12) | line numbers and needles identical |
| `moveit-smoothing/src/{acceleration_filter.rs,butterworth.rs,ruckig_filter.rs}` (11 sites) | `contains_msg` | discriminating (11/11) | line numbers and needles identical |

No source changes this round — the population is unchanged from round
11, so its 0-blind-sites result stands unmodified. This section exists
to correct the reported 81 with worked arithmetic, not to reopen the
sweep.

### Commands run (round 12)

- `git merge main` (already run before this session's summary point) — merge commit `63e043a`, pulled in `ccac7ea`, census §9e, `doc/assertion-discrimination-ledger-p9-ros.md`
- `python3 tools/ci/count-coarse-assertions.py crates 2>/dev/null | rg -v 'moveit-planners-pilz' > tool_out.txt` — 548 total lines, all kinds
- `rg ':contains_msg:' tool_out.txt | wc -l` → 65; `rg ':contains_msg:' tool_out.txt | rg -v ':helper_body:' | wc -l` → 64
- `rg ':via:' tool_out.txt | wc -l` → 12; broken down by helper name (`sed -E 's/^.*:via:([A-Za-z_0-9]+):.*$/\1/' | sort | uniq -c`) → `assert_err_mentions` 8, `check_scenario` 2, `rows_to_string` 2
- `rg ':helper_body:' tool_out.txt` → 4 lines, read each helper body to classify family membership
- `rg 'robot_model.rs:(2472|2512|2542):' tool_out.txt` and `rg 'check_start_state_collision.rs:(161|162):' tool_out.txt` — confirmed still `contains_member`, blind spot unfixed by `ccac7ea`
- `cargo fmt --all -- --check` — clean

Gate scope: doc-only round, `cargo fmt --all -- --check`. No source
file in this fence changed, so no `-p <crate>` clippy/nextest gate is
owed.

## Round 13 — path-based fence, `f10e1bd`'s unified `contains`, census §9 membership

`git merge main` (merge commit `1fa8d9f`) pulled in `f10e1bd`
("collapse `count-coarse-assertions.py`'s `contains_msg`/`contains_member`
split" — round 11/12's `detail`-field blind spot forced this: a
receiver-type distinction has no regex, and the alternative rule tried
misfiled `body.touch_links().contains(...)` at `ros/moveit-ros/src/scene/attached.rs:532`)
and `6792ef1` ("score `None`/`Err` only as an operand of an equality
macro"). The tool now emits one `contains` kind and leaves membership to
census §9 clause 1, by reading — exactly what this round does. Fence
reassigned by path: `robot_model.rs`, `mesh_search_paths.rs`,
`robot_model_parity.rs`, `joint/model.rs`, `decide.rs`,
`moveit-planning/`.

### The stated 41 does not survive enumeration; the sum of the same six
### per-path figures is 52

`python3 tools/ci/count-coarse-assertions.py <the six paths> | rg -v
'moveit-planners-pilz'` (pilz not among these paths anyway), then kept
every hit whose `kind` is not exactly `matches`/`is_err`/`is_none` and
whose `scope` is not `helper_body`:

| path | count |
|---|---|
| `robot_model.rs` | 24 |
| `mesh_search_paths.rs` | 5 |
| `robot_model_parity.rs` | 4 |
| `joint/model.rs` | 3 |
| `decide.rs` | 11 |
| `moveit-planning` (`check_start_state_bounds.rs` 3 + `check_start_state_collision.rs` 2) | 5 |
| **total** | **52** |

Every one of these six numbers matches the brief's own per-path figures
exactly — the breakdown was right. `24+5+4+3+11+5 = 52`, not 41; the
brief's total was a bad sum over its own correct rows, not a bad
enumeration. One `helper_body` line excluded (`decide.rs:83`,
`assert_err_mentions`'s own body).

### Census §9 membership, per site (52 sites)

**`crates/moveit-constraints/tests/decide.rs` (11)**

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `decide.rs:362,380,438,508,534,553,742,781` (`via:assert_err_mentions`) | yes | discriminating | round 11's sibling-message table, unchanged (same 8 call sites) |
| `decide.rs:1161,1166` (`assert_eq!(..., None)` on `max_view_angle()`/`max_range_angle()`, both inside `negative_target_radius_activates_at_its_magnitude_but_negative_angles_stay_inactive`) | yes | discriminating | isolating mutation run this round: `normalize_angle_criterion`'s `.filter(\|v\| *v > EPS)` gate removed → `negative_target_radius_activates_at_its_magnitude_but_negative_angles_stay_inactive` and `zero_valued_criteria_normalize_to_unconstrained` both fail (2/100), no other test moves; `target_radius`'s own gate (`normalize_target_radius`, a different fn) is untouched and its sibling assertion in the same test stays green |
| `decide.rs:1257` (`set.is_empty()`) | **no** | not-this-family | `KinematicConstraintSet::new()` is `Self::default()` (`crates/moveit-constraints/src/set.rs:54-56`) — trivially empty by construction, clause 2 fails (nothing was decided) |

**`crates/moveit-model/src/joint/model.rs` (3)**

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `crates/moveit-model/src/joint/model.rs:946` (`local_variable_names().is_empty()`, single-variable joint, inside `single_variable_joints_use_the_bare_joint_name`) | yes | discriminating | cross-test sibling `multi_variable_joints_prefix_local_names_with_the_joint_name` shows 7 non-empty names for a floating joint |
| `crates/moveit-model/src/joint/model.rs:975` (`variable_names().is_empty()`, fixed joint) | yes | discriminating | cross-test siblings show 1 name (revolute, `:944`) and 7 names (floating, `:948-955`) |
| `crates/moveit-model/src/joint/model.rs:976` (`variable_bounds().is_empty()`, fixed joint) | yes | discriminating | cross-test siblings index `variable_bounds()[0]` directly at `:1008,1031,1074,1087,1105` — non-empty for every other joint kind tested |

**`crates/moveit-model/src/mesh_search_paths.rs` (5)**

`resolve()` has four independent `None`-producing guards (`?` on
`strip_prefix("package://")`, `?` on `split_once('/')`, `?` on
`self.packages.get(package)`, and `candidate.is_file()` false).

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `mesh_search_paths.rs:108` (`paths.is_empty()`, on `none()`) | **no** | not-this-family | `is_empty()` on `packages.is_empty()`, `none()` on `Self::default()` — both trivial, no decision links them |
| `mesh_search_paths.rs:109` (`none_resolves_nothing`) | yes | discriminating | empty map — isolates the `self.packages.get(package)?` guard; distinguishable from `:139/140`'s guard (see below) |
| `mesh_search_paths.rs:141` (`unknown_package_does_not_resolve`) | yes | discriminating | map has one real entry, queried key absent — same guard as `:109` but proves it is the lookup miss, not "map empty," that fires — line corrected round 17, was line 130 |
| `mesh_search_paths.rs:162,163` (`non_package_uri_does_not_resolve`) | yes | discriminating | neither input has a `"package://"` prefix — isolates `strip_prefix(...)?`, distinct guard from `:109/130` |

**Not fixed, flagged rather than fabricated**: the `split_once('/')?`
guard (malformed `package://name` with no `/`) and the
`candidate.is_file()` guard (known package, well-formed path, file
missing) had **no test at all** — not a blind existing assertion, an
absent one. Writing new tests was outside "fix the blind ones"; this
was a coverage gap, not a fix owed that round. Reporting it rather than
silently dropping it.

**Both are tested now, and round 14 below is where that happened.**
`b64806d6` (`test(moveit-model): prove mesh_search_paths::resolve's four
None guards actually isolate`) added
`malformed_package_uri_with_no_relative_path_does_not_resolve`
(`crates/moveit-model/src/mesh_search_paths.rs:174`) for
`rest.split_once('/')?`
(`crates/moveit-model/src/mesh_search_paths.rs:87`) and
`missing_file_does_not_resolve`
(`crates/moveit-model/src/mesh_search_paths.rs:189`) for
`candidate.is_file().then_some(candidate)`
(`crates/moveit-model/src/mesh_search_paths.rs:89`), each carrying its
own isolating-mutation record. The gap this paragraph opened was closed
one round later; the paragraph is the round-13 record of it, not a live
item.

**`crates/moveit-model/src/robot_model.rs` (24)**

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `robot_model.rs:2013` (`mimic_chain_collapses_transitively`, diagnostics-empty) | yes | discriminating | cross-test siblings: `MimicCycle` positive at `:2051`, `MimicUnknownJoint` at `:2073`, `MimicDofMismatch` at `:2105` |
| `robot_model.rs:2301` (`only_j1`'s `subgroup_names().is_empty()`) | yes | discriminating | real per-candidate comparison against `only_j2`/`all` (not vacuous — candidates exist); same test's `all.subgroup_names()` (`:2259`) is the non-empty sibling |
| `robot_model.rs:2328,2343` | yes | discriminating | round 11, unchanged |
| `robot_model.rs:2410` (box collision, diagnostics-empty) | yes | discriminating | cross-test siblings: capsule/mesh/negative-dimension diagnostic tests below it in the same file |
| `robot_model.rs:2433` (`base.shapes().is_empty()`, link with zero `<collision>` elements) | **no** | not-this-family | vacuous — `link_with_geometry_urdf("")` has no collision element at all, nothing for the shape-construction loop to iterate; same shape as the census's `shortest_solution_is_none_on_empty_input` example |
| `robot_model.rs:2452,2559` (mesh skipped, `shapes().is_empty()`) | yes | discriminating | real skip guard (path resolution/unreadable file), cross-test sibling `:2561-2564` shows `shapes.len()==1`, `Shape::Mesh(_)` when resolution succeeds |
| `robot_model.rs:2513,2553,2583` | yes | discriminating | round 11, unchanged |
| `robot_model.rs:2602` (valid stl, diagnostics-empty) | yes | discriminating | positive-case anchor for `:2411/2518/2472/2512/2542`; cross-test siblings show diagnostics populated for every failure shape |
| `robot_model.rs:2676` | yes | discriminating, breadth caveat | round 11, unchanged |
| `robot_model.rs:2717` (`visual_mesh_filename()`, `None`) | yes | discriminating | cross-test siblings `:2650` (`Some("visual.dae")`), `:2666` (`Some("collision.stl")`) |
| `robot_model.rs:2822` (`"other"` group, `attached_end_effector_names().is_empty()`) | yes | discriminating | same test: `"arm"`/`"full_arm"` (`:2755,2762`) are the non-empty siblings; real ancestry-walk decision, not vacuous — line corrected round 17, was line 2764 |
| `robot_model.rs:2943` (`end_effector_parent()`, `None`) | **no** (corrected round 15) | single-branch, excluded | field default, set only by `set_end_effector_parent`, never called in this fixture — see round 15's "Excluded: single guard" table below; superseded the cross-test-sibling reasoning originally in this row — line corrected round 17, was line 2885 |
| `robot_model.rs:2944` (`attached_end_effector_names().is_empty()`, no end effector declared at all) | **no** (corrected round 16) | not-this-family | vacuous — this test's SRDF declares zero end effectors at all, so nothing ever populates this field for any group; the original cross-test-sibling reasoning here compared against a *different* test's fixture, not this test's own causal chain — see round 16 — line corrected round 17, was line 2886 |
| `robot_model.rs:3044` (unknown-group `group_state`, ignored) | yes | discriminating, **bitten round 16** | real group-name-lookup guard, distinct decision from `:3038`'s; round 16 replaced the original reading-only cross-test-sibling evidence with an isolating mutation on `build_group_states` (`:1600` vs `:1629`) — line corrected round 17, was line 2986 |
| `robot_model.rs:3079` (every joint value unusable, no state stored) | yes | discriminating, **bitten round 16** | real per-value dimension-mismatch guard (not vacuous — one `<group_state>` element present); round 16 isolating mutation, see above — line corrected round 17, was line 3021 |
| `robot_model.rs:3089` (`variable_default_positions_returns_none_for_unknown_state_name`, empty srdf) | **no** | not-this-family | vacuous — `group_state_test_srdf("")` has zero `<group_state>` elements, nothing for the collection loop to iterate — line corrected round 17, was line 3031 |
| `robot_model.rs:3468,3469,3470,3471` (fixed-joints-only group: `active_joint_indices`/`joint_roots`/`updated_link_names`/`updated_link_with_geometry_names`, all empty) | yes | **funnel, closed round 18** | round 18 isolating mutation proved this row's own "single-branch, not vacuous" call wrong — a neutralized producer left this exact test green; closed by adding a same-test `"arm"` sibling group with a real active joint whose non-empty `joint_roots`/`updated_link_names` prove `compute_group_topology`'s per-group closure ran — see the round-18 table above for the full mutation/closure evidence — line corrected round 17 (was lines 3376-3379) then round 18 (was lines 3393-3396, shifted by this round's fixture edit) |

**`crates/moveit-model/tests/robot_model_parity.rs` (4)**

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `robot_model_parity.rs:316` (`build_clean_model_with_urdf`, `srdf.diagnostics().is_empty()`) | **no** | not-this-family | message says it outright: `"fixture SRDF must parse cleanly"` — a precondition on this test's own fixture file, not on `RobotModel` (the subject `assert_matches_oracle` verifies); clause 3 |
| `robot_model_parity.rs:356` (`is_end_effector()` vs oracle's `is_some()`) | **no** | not-this-family | `is_end_effector()` is a computed classification tag compared directly to an oracle value — clause 1, same exclusion as `matches!(kind(), Fixed)` in the census's own worked example |
| `robot_model_parity.rs:361` (`end_effector_name()`-derived `Option` vs oracle) | **no** | not-this-family | same as `:356` — oracle-parity comparison of a computed value, not a coarse fail/absence signal masking a subject decision |
| `robot_model_parity.rs:518` (`unsupported_geometry_links.contains(name)`) | yes | discriminating | guarded by `if !group.updated_link_with_geometry_names().contains(name)` — asserts the discrepancy is explained by the unsupported-geometry diagnostic specifically, not left unexplained; message makes the discrimination explicit |

**`crates/moveit-planning/` (5)**

`RequestAdapterError::StartStateInvalid { adapter }` carries no detail
field; `CheckStartStateBounds::adapt`'s final guard
(`check_start_state_bounds.rs:169`) is `is_out_of_bounds ||
(!fix_start_state && should_fix_state)`, itself folding two named
operands, and `is_out_of_bounds` is set by two further independent
guards (position bounds, velocity bounds) inside the per-joint loop —
three real causes producing the identical `Err` value.

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `check_start_state_bounds.rs:358` (`an_out_of_bounds_velocity_is_rejected`) | yes | discriminating | **isolating mutation, run this round**: `if false && state.has_velocities() && !joint.satisfies_velocity_bounds(...)` → this test fails (`Ok(())` vs expected `Err`), `:384`'s test stays green — line corrected round 17, was line 302 |
| `check_start_state_bounds.rs:384` (`a_joint_placed_outside_its_limits_is_rejected_regardless_of_fix_start_state`) | yes | discriminating | **isolating mutation, run this round**: `if false && !joint.satisfies_position_bounds(...)` → this test fails, `:358`'s test stays green — line corrected round 17, was line 328 |
| `check_start_state_bounds.rs:433` (`(-PI..=PI).contains(&wrapped)`) | **no** | not-this-family | range check on a directly computed numeric result (the wrap itself), not an absence/failure signal — same exclusion as `sampler.rs`'s bounds checks in round 11 — line corrected round 17, was line 377 |
| `check_start_state_collision.rs:161,162` | yes | discriminating | round 11, unchanged |

`:169`'s guard actually folds a *third* operand
(`!fix_start_state && should_fix_state`, exercised by
`a_continuous_joint_past_pi_is_wrapped_and_accepted_only_when_fix_start_state_is_set`)
that produces the same `Err` value but is not itself one of the 52
enumerated sites (its own assertion is an `eq_err`-shaped check outside
this file's three flagged lines — the wrap test's assertion is at a
different line not in this round's `contains`/`eq_err`/`eq_none`/etc.
grammar). Neutralized it anyway for completeness, since all three
operands share one construction site: isolating mutation confirms it
too fails only its own test, `:302`/`:328` staying green. All three
mutations were applied, run with `--no-fail-fast`, confirmed, and
reverted; `git status --short` is clean, no residual diff.

### Result: 0 blind sites; 8 of 52 ruled `not-this-family`; one coverage gap flagged, closed by round 14 (`b64806d6`)

44 of 52 sites are family members: **40 `discriminating`** (8 of those
are `via:assert_err_mentions`'s call sites) and **4 `single-branch`**
(the `group_of_only_fixed_joints` cluster, `robot_model.rs:3376-3379`).
**8 are `not-this-family`**: `decide.rs:1257`,
`mesh_search_paths.rs:108`, `robot_model.rs:2392,3031`,
`robot_model_parity.rs:316,356,361`,
`check_start_state_bounds.rs:377`. 44 + 8 = 52. No member's needle
collided with a sibling's — 0 blind sites, so 0 fixes owed.

Three isolating mutations were run empirically this round (the
`normalize_angle_criterion` EPS gate, and two of
`CheckStartStateBounds`'s three folded guards — position, velocity,
and the wrap-reject operand) because their shared, detail-less
`Err`/`None` return values made them the highest-risk sites in this
batch: multiple real causes producing byte-identical results. All
three isolated cleanly (guard neutralized → exactly its own test
fails, siblings stay green), so all three are `discriminating`, not
blind. Every other verdict rests on reading plus cross-test sibling
citation, the same evidentiary bar round 11 used.

### Commands run (round 13)

- `git merge main` — merge commit `1fa8d9f`, pulled in `f10e1bd`, `6792ef1`, census §9's expanded text
- `python3 tools/ci/count-coarse-assertions.py <six paths> > round13_raw.txt` (paths: `robot_model.rs`, `mesh_search_paths.rs`, `robot_model_parity.rs`, `joint/model.rs`, `decide.rs`, `moveit-planning`)
- Python one-off (`re.match(r'^(.*?):(\d+):(.*?):(test|src|helper_body):(.*)$', ...)`) to split kind/scope correctly — naive `awk -F:` breaks on `via:<fn>`'s embedded colon
- Per-file counts cross-checked against the brief's own six numbers — exact match; total re-summed by hand: 52
- `rg 'robot_model.rs:(2472|2512|2542):'`, `check_start_state_collision.rs:(161|162):` — reconfirmed still `contains`/unresolved by `f10e1bd` in the sense that both sites are correctly members regardless of kind label now
- Isolating mutation 1: `crates/moveit-constraints/src/visibility.rs`'s `normalize_angle_criterion` — `cargo nextest run -p moveit-constraints --no-fail-fast`, 2/100 failed, reverted
- Isolating mutations 2-4: `crates/moveit-planning/.../check_start_state_bounds.rs`'s position guard, velocity guard, and the fold's third operand, each individually — `cargo nextest run -p moveit-planning check_start_state_bounds --no-fail-fast`, exactly one test failed each time, reverted
- `git status --short` / `git diff --stat` — clean after every revert
- `cargo fmt --all -- --check` — clean

Gate scope: no source file left changed this round (0 fixes — all
mutations reverted after use), so `-p moveit-model -p moveit-constraints
-p moveit-planning` clippy/nextest were exercised only incidentally by
the mutation-verification runs above, not as a post-fix gate. Doc-only
commit; `cargo fmt --all -- --check` is what's owed and was run.

## Round 14: `mesh_search_paths.rs`'s two "tested" guards were blind — round 13's verdicts for `:109,130,139,140` corrected

The user overruled round 13's "outside the scope of 'fix the blind
ones'" framing of the coverage gap flagged above, on evidence: reading
plus cross-test input-disjointness is an argument about the fixture,
not about the assertion, and is not the isolating-mutation evidence bar
this sweep otherwise holds itself to. Ran the isolating mutation on the
two guards round 13 had marked `discriminating` from reading alone.

**Both were blind.** `resolve()`'s four `?`/bool guards
(`strip_prefix`, `split_once`, `packages.get`, `is_file`) all produce
the same payload-less `None`, and the two negative tests that existed
(`unknown_package_does_not_resolve`, `non_package_uri_does_not_resolve`)
built fixtures with no real file at the joined path. Neutralizing
either `strip_prefix` or `packages.get` alone left every original test
green: the chain simply fell through to `candidate.is_file()` failing
for an unrelated reason (no file on disk), and that later guard's
`None` masked the neutralized guard's own `None`. Round 13's `:130`
(`packages.get`) and `:139,140` (`strip_prefix`) verdicts are corrected
from `discriminating` to **blind** — self-diagnosed by this round's
own standard, not merely reclassified by re-reading.

`:109` (`none_resolves_nothing`, empty map) is corrected from
`discriminating` to **single-branch**: with `packages` genuinely
empty, `packages.get` returns `None` regardless of key, and there is no
possible fallback candidate to substitute — the guard's neutralized
form (`.or_else(|| packages.values().next())`) still returns `None` on
an empty map, so this input cannot separate "guard fired" from "guard
absent." It is a legitimate single-cause site (config precondition
`is_empty()`, distinct from `:130`'s "map non-empty, key absent"),
not a discriminated one.

### Fix: shared real-file fixture, four isolating mutations, all clean

`Option<PathBuf>`'s four causes ARE separable by input alone — the
existing positive test (`resolves_against_the_mapped_package_directory`)
already proves the chain resolves to `Some` when a real file exists at
the joined path, so a fixture built around that same real file, with
exactly one guard's neutralized-fallback landing on it, isolates each
guard cleanly. The user's step-3 escape valve ("if all four cannot be
separated by input alone... `Option<PathBuf>` may simply be too coarse
a return type") does **not** apply; no signature change is proposed.

Rewrote `unknown_package_does_not_resolve` and
`non_package_uri_does_not_resolve` so both share a fixture with a real
file at `<dir>/meshes/link0.stl` under the one registered package
`moveit_resources_panda_description`, and added two new tests
(`malformed_package_uri_with_no_relative_path_does_not_resolve` for
`:87`, `missing_file_does_not_resolve` for `:89`) built the same way.
Each mutation neutralizes exactly one guard with a fallback that lands
on that real file, so a truly-isolated guard flips `None` → `Some`:

| guard | mutation | result |
|---|---|---|
| `:86` `strip_prefix("package://")?` | bypassed to use `resource` unstripped | exactly `non_package_uri_does_not_resolve` failed (`Some(.../meshes/link0.stl)` vs expected `None`); 5 siblings green |
| `:87` `split_once('/')?` | bypassed with a fallback relative path `"meshes/link0.stl"` | exactly `malformed_package_uri_with_no_relative_path_does_not_resolve` failed; 5 siblings green |
| `:88` `packages.get(package)?` | `.or_else(\|\| packages.values().next())` | exactly `unknown_package_does_not_resolve` failed; 5 siblings green (`none_resolves_nothing` correctly stayed green — empty map, no fallback exists) |
| `:89` `candidate.is_file()` | `(candidate.is_file() \|\| true)` | exactly `missing_file_does_not_resolve` failed; 5 siblings green |

All four run with `cargo nextest run -p moveit-model mesh_search_paths
--no-fail-fast`, one at a time, each reverted before the next. Final
`resolve()` body is byte-identical to the pre-round original; `git
diff --stat` shows only the `#[cfg(test)] mod tests` block changed.

### Corrected verdict table — `mesh_search_paths.rs` (supersedes round 13's four rows)

| file:line | member? | verdict (round 13) | verdict (round 14, corrected) | evidence |
|---|---|---|---|---|
| `mesh_search_paths.rs:108` (`paths.is_empty()`, on `none()`) | **no** | not-this-family | not-this-family (unchanged) | trivial getter, no decision |
| `mesh_search_paths.rs:109` (`none_resolves_nothing`) | yes | discriminating | **single-branch** | empty map has no fallback candidate; guard cannot be further isolated by input |
| `mesh_search_paths.rs:141` (`unknown_package_does_not_resolve`) | yes | discriminating | **corrected: was blind, now discriminating** | isolating mutation on `:88` (see table above) — round 13's verdict rested on reading, not mutation, and was wrong until the fixture was rebuilt — line corrected round 17, was line 130 |
| `mesh_search_paths.rs:162,163` (`non_package_uri_does_not_resolve`) | yes | discriminating | **corrected: was blind, now discriminating** | isolating mutation on `:86` (see table above) — same correction |
| `mesh_search_paths.rs` (`malformed_package_uri_with_no_relative_path_does_not_resolve`, new) | yes | — (did not exist) | discriminating | isolating mutation on `:87` |
| `mesh_search_paths.rs` (`missing_file_does_not_resolve`, new) | yes | — (did not exist) | discriminating | isolating mutation on `:89` |

### Result

All four guards in `resolve()` are now covered by a test that has been
proven, by actual mutation, to fail if and only if that guard is the
one that fired. `:109` remains the sole `single-branch` site in this
file — a real, defensible distinction from `:130`, not a downgrade
applied uniformly. 0 blind sites remain in `mesh_search_paths.rs`.

### Commands run (round 14)

- `cargo nextest run -p moveit-model mesh_search_paths --no-fail-fast` — 4 isolating-mutation runs (`:86`, `:87`, `:88`, `:89`), each showing exactly one failure, each reverted immediately after
- `git diff --stat` after final revert — only the test module changed, `resolve()` body identical to pre-round
- `cargo fmt --all`
- `cargo clippy -p moveit-model --all-targets -- -D warnings` — clean
- `cargo nextest run -p moveit-model` (full crate) — 136 tests run, 136 passed, 0 skipped

Gate scope: `-p moveit-model`, source-touching round, all three steps
(`fmt`, `clippy -D warnings`, `nextest`) run and clean as owed.

## Round 15: audit the 53-site fence itself for the funnel shape census §9g named

`mesh_search_paths.rs`'s bug is now census §9g. The user's charge this round
was not to re-derive membership again but to hunt the *same defect shape*
across the rest of the fence: every site whose subject routes two or more
distinguishable guards into one undifferentiated `None`/`Err`, verdicted
`discriminating` by reading. Re-ran `count-coarse-assertions.py` against the
same six paths first — 53, not 52 (round 14 added two `eq_none` sites of its
own, `mesh_search_paths.rs:179,192`; every other line number below is
current against `HEAD` at merge `3da1308`).

### Method

For every site, read the *subject* function (not the test) for a chain of
`?`/early-return/boolean-fold guards converging on one bare value with no
distinguishing payload. Two found; both bitten. Everything else is excluded
below, by class, with the specific reason it cannot exhibit `resolve()`'s
defect shape.

### Funnel 1: `CheckStartStateBounds::adapt`'s `should_fix_state`/`is_out_of_bounds` — 5 causes, one bare `Err`, 2 of 5 untested

`check_start_state_bounds.rs:169`'s guard (`is_out_of_bounds ||
(!fix_start_state && should_fix_state)`) is fed by five independent
per-joint causes, all folding into the identical
`Err(RequestAdapterError::StartStateInvalid { adapter })` — no detail field
to distinguish them:

- `is_out_of_bounds`: position bounds (`:157`), velocity bounds (`:160`)
- `should_fix_state`: continuous-revolute wrap (`:138`), planar rotation
  normalize (`:144`), floating quaternion normalize (`:150`) — all three
  named explicitly in the module doc (steps 1-3) as of this file's own
  design, not a fact this round discovered

Round 13 bit position and velocity (`:302`, `:328`, both in-fence `eq_err`
sites) and, incidentally, the continuous-revolute-wrap cause via a test
outside the 53-site grammar. **Planar and floating rotation-normalize had
no test anywhere in the file** — not blind assertions, absent ones, exactly
`mesh_search_paths.rs:87,89`'s shape before round 14.

**Fixed.** Added `planar_robot()`/`floating_robot()` fixtures (neither
`panda` nor `pr2` has a planar or floating joint) and two new tests,
mirroring the existing continuous-wrap test's two-phase shape (`fix_start_state
= false` rejects, `= true` accepts and the representation is normalized).
Re-verified all five causes by isolating mutation this round, not carried
forward from round 13's report:

| cause | mutation | result |
|---|---|---|
| position (`:157`) | `!joint.satisfies_position_bounds(...) && false` | exactly `a_joint_placed_outside_its_limits_is_rejected_regardless_of_fix_start_state` failed; 7 siblings green |
| velocity (`:160`) | `... && false` appended to the velocity condition | exactly `an_out_of_bounds_velocity_is_rejected` failed; 7 siblings green |
| continuous wrap (`:138`) | `joint.enforce_position_bounds(&mut values) && false` | exactly `a_continuous_joint_past_pi_is_wrapped_...` failed; 7 siblings green |
| planar normalize (`:144`), new | `PlanarJoint::normalize_rotation(planar) && false` | exactly `a_planar_joint_past_pi_is_wrapped_...` (new test) failed; 7 siblings green |
| floating normalize (`:150`), new | `FloatingJoint::normalize_rotation(floating) && false` | exactly `a_floating_joint_with_a_non_unit_quaternion_...` (new test) failed; 7 siblings green |

All five run one at a time with `cargo nextest run -p moveit-planning
check_start_state_bounds --no-fail-fast`, each reverted before the next.
`git diff --stat` after final revert shows only the `#[cfg(test)] mod
tests` block changed; `adapt()`'s body is byte-identical to pre-round.

### Funnel 2: `visual_mesh_filename()` — 2 causes, one bare `None`, 1 of 2 untested

`RobotModel`'s per-link visual-mesh resolution (`robot_model.rs:1189-1212` (`apply_link_geometry`))
has two distinguishable ways to leave `visual_mesh_filename()` at `None`:

- no `<mesh>` geometry found in either `<visual>` or `<collision>` (the
  `.filter(is_mesh).or_else(...)` chain itself yields `None`) — tested,
  `no_mesh_in_visual_or_collision_leaves_visual_mesh_filename_none`
- a `<mesh>` element *is* found, but `filename` is an empty string
  (`!filename.is_empty()` at `:1202` guards the `set_visual_mesh` call) —
  **untested**

Unlike `resolve()`'s `?`-chain, these two causes are not sequential
fallthrough — they sit on disjoint branches of `if let Some(mesh) { if
!empty { .. } }`, and the outer branch is a refutable enum-variant match
(`Geometry::Mesh{..}`) that a non-mesh geometry can never satisfy regardless
of the inner guard's state. So the existing test cannot ride the untested
guard the way `mesh_search_paths.rs`'s two tests rode each other. Verified,
not merely reasoned: bypassing the inner guard
(`!filename.is_empty() || true`) left the existing "no mesh" test green and
failed only the new one.

**Fixed.** Added `mesh_with_an_empty_filename_leaves_visual_mesh_filename_none`
(`<visual><geometry><mesh filename=""/></geometry></visual>`). Isolating
mutation: `!filename.is_empty()` → `!filename.is_empty() || true` — exactly
the new test failed (`Some("")` vs expected `None`), the existing
"no mesh found" test stayed green, confirmed via `cargo nextest run -p
moveit-model visual_mesh --no-fail-fast`. Reverted; `git diff` on
`robot_model.rs`'s `resolve_...`/mesh-construction code shows only the new
`#[test]` fn added.

### Excluded: single guard (name and reason)

| site | subject | why not a funnel |
|---|---|---|
| `crates/moveit-model/src/joint/model.rs:946` (`local_variable_names().is_empty()`) | `JointModel::new_single_variable` | unconditional `Vec::new()` in one constructor body — every single-variable joint kind (revolute/prismatic/continuous) routes through this one function, not several guards |
| `crates/moveit-model/src/joint/model.rs:975,976` (`variable_names`/`variable_bounds().is_empty()`) | `JointModel::new_fixed` | same shape, unconditional `Vec::new()`, one constructor |
| `decide.rs:1161,1166` (`max_view_angle`/`max_range_angle`, bare `None`) | `normalize_angle_criterion` = `value.filter(\|v\| *v > EPS)` | one guard (`> EPS`); the other way to reach `None` is the input already being `None` (criterion unset), which is `Option` pass-through, not an independent application decision — same shape as `mesh_search_paths.rs:109` (`none_resolves_nothing`)'s excluded single-branch case |
| `robot_model.rs:2943` (`end_effector_parent()`, bare `None`) | field default, set only by `set_end_effector_parent` | reachable *only* by that call never happening (group never named as an `<end_effector>` `component_group`); `build_end_effectors`'s own 3-cause funnel (explicit-parent self-reference, explicit-parent lacking the link, no fallback candidate) produces `Some(EndEffectorParent{group: None, ..})`, a distinguishable payload, not this bare `None` — and is not itself one of the 53 sites (not an `eq_none`-shaped assertion) — line corrected round 17, was line 2885 |

### Excluded: distinguishable payload, not undifferentiated

| site(s) | reason |
|---|---|
| `decide.rs`'s 8 `via:assert_err_mentions` sites | needle-matched by design — the helper's own doc comment states its purpose is exactly to prevent this defect shape for `.is_err()`-style checks |
| `robot_model.rs:2328,2343,2513,2553,2583,2676` | each `.contains()` checks a distinct message substring unique to its guard |
| `robot_model_parity.rs:518` | set membership by link *name*, not a bare `Err`/`None` |
| `check_start_state_collision.rs:161,162` | distinct message substrings (`"contact(s) detected"`, `"engulfing_box"`) |

### Excluded: `Vec::is_empty()` class, `:2411,:2518` cleared round-16-precedent, remaining 9 audited round 18

`robot_model.rs:1972,2260,2369,2411,2518,2561,2781,3427,3428,3429,3430` (11
sites total, current line numbers — the last four were lines 3393-3396 before
this round's fixture edit, see the `:3427...` row below). `:2392,3048`
already `not-this-family`; `:2903` (old `:2886`)
also reclassified `not-this-family` round 16 — its old cross-test-sibling
evidence compared a different test's fixture, not this one's own causal
chain, see the round 13 table above.

The blanket argument that used to cover all 11 ("*any* push, from *any*
cause, flips the result to non-empty") is **retired as of round 18** — it
answers a different question (could a wrong result be produced) than the
one that matters here (did the subject run at all, or was it skipped
early). `:2411`/`:2518` (the mesh-skip tests) were already cleared on the
sharper standard: each carries its own sibling `detail`-string assertion
in the same test that independently names which failure branch fired,
making the `is_empty()` a secondary confirmation, not the sole evidence.
The other 9, re-derived fresh from `count-coarse-assertions.py` (not
trusted from this ledger's own prior numbers, per round-18 correction) and
audited one by one below — no site is closed by "any push" alone; each
verdict is site-specific.

| site | subject | class | reason |
|---|---|---|---|
| `robot_model.rs:2013` (`mimic_chain_collapses_transitively`) | `resolve_mimic`'s diagnostic-producing loop | (a) | sibling `assert_eq!(mimic.joint_name, "j1")` (`:1976`), `mimic.factor, 6.0` (`:1977`), `mimic.offset, 1.6` (`:1978`) in the same test. j3's collapsed factor/offset (6.0 = 3.0×2.0, 1.6 = 0.1+3.0×0.5) match no raw URDF literal (j2 mimic: 2.0/0.5; j3 mimic: 3.0/0.1) — they are producible only by `resolve_mimic`'s second (chain-collapse) loop actually running its arithmetic on j3's still-live mimic record, which itself requires the first (diagnostic) loop to have reached and passed j2/j3 without clearing either mimic. "The diagnostic checks never ran" cannot explain these exact values. |
| `robot_model.rs:2410` (`box_collision_at_identity_produces_a_shape_and_a_centered_bounding_box`) | link-geometry shape parsing | (a) | sibling `assert_eq!(base.shapes(), [LinkShape { shape: Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0)...` (`:2372-2378`) in the same test. The asserted cuboid dims (2.0, 4.0, 6.0) are the literal `<box size="2 4 6"/>` values from the fixture — reachable only if shape-construction actually parsed this element, ruling out "geometry parsing never ran" as an explanation for empty diagnostics. |
| `robot_model.rs:2602` (`mesh_collision_resolving_to_a_valid_stl_file_builds_a_mesh_shape`) | mesh-file parsing | (a) | sibling `assert_eq!(shapes.len(), 1)` (`:2604`) and `assert!(matches!(shapes[0].shape, Shape::Mesh(_)))` (`:2605`) in the same test, against an STL actually written to disk by the test (`synthetic_binary_stl()`). A `Shape::Mesh` only exists if the binary STL was read and parsed; "mesh parsing never ran" would leave `shapes` empty, not len 1 with a mesh variant. |
| `robot_model.rs:2301` (`subgroup_detection_lists_every_strict_subset_alphabetically`) | `compute_subgroups` (`:1684-1716`) | (a) | sibling `assert_eq!(all.subgroup_names(), ["only_j1", "only_j2"])` (`:2259`) in the same test, same function, same `build()` call. Read `compute_subgroups`: the outer `for name in &names` loop is unconditional — every group unconditionally gets a `subgroup_names_by_group.insert(name.clone(), subgroup_names)` (`:1708`), no group-name-gated skip exists. The `:2259` sibling proves this loop executed for this model; since it is un-gated per-group, it executed identically for `"only_j1"`. `"only_j1"`'s empty result is then just subset math: neither `"all"` (superset) nor `"only_j2"` (disjoint {j2}) is a subset of {j1}. |
| `robot_model.rs:2822` (`end_effector_wires_name_and_falls_back_to_fewest_joints_parent`) | `build_end_effectors`'s inner `for other_name in &group_names` loop (`:1745-1807`) | (a) | sibling `assert_eq!(arm.attached_end_effector_names(), ["grasper"])` (`:2767-2773`) and `assert_eq!(full_arm.attached_end_effector_names(), ["grasper"])` (`:2774-2780`) in the same test. Both pushes happen inside the single un-broken `for other_name in &group_names` loop run for eef `"grasper"` (matched once, at `group_name == "hand"`); that loop has no `break` and visits every `other_name` including `"other"`. The sibling pushes prove this exact loop-instance ran to completion; `"other"`'s empty result follows because its only joint (`j5`) doesn't contain `link2` (`end_effector_test_srdf`, `:2743`), not because the loop skipped it. |
| `robot_model.rs:3468,3469,3470,3471` (was lines 3393-3396 before this round's fixture edit; `group_of_only_fixed_joints_has_no_joint_roots_or_updated_links`) | `make_joint_model_group`'s active/fixed split (`:1486-1501`) feeding `compute_group_topology`/`group_joint_roots`/`group_updated_links` (`:672-746`) | (c) | **genuine funnel, closed this round.** All four fields are `Vec::new()` at construction (`:1513,1527,1529,1531`) and stay that value under real computation too, since `"rigid"` has no active joint — no same-test sibling on `"rigid"` itself can ever discriminate "ran, correctly empty" from "never ran". Proved by isolating mutation: added `&& !true` to `make_joint_model_group`'s `if node.model.mimic().is_none()` guard (so no joint anywhere is ever classed active) — `group_of_only_fixed_joints_has_no_joint_roots_or_updated_links` **stayed green** while `joint_roots_lists_every_root_of_a_multi_rooted_group`, `updated_link_names_filters_to_geometry_bearing_links`, `is_chain_true_across_an_unlisted_fixed_joint`, and all four `robot_model_parity` oracle tests failed (`cargo nextest run -p moveit-model --no-fail-fast`; reverted). Closed by adding a second SRDF group `"arm"` (one active joint, `j3`) to the same fixture and asserting `arm.joint_roots() == [j3]` / `arm.updated_link_names() == ["arm_tip"]` in the same test. `compute_group_topology`'s per-group closure (`:672-689`) and its write-back loop (`:690-713`) have no group-name conditional — confirmed by reading both — nor do `group_joint_roots`/`group_updated_links` (their only filters are per-joint/per-link). `"arm"`'s non-empty result is therefore proof the same, uniformly-applied closure executed for `"rigid"` too in the same `build()` call. Re-ran the identical mutation against the fixed test: it now **fails** at the new `arm.joint_roots()` assertion (`cargo nextest run -p moveit-model --no-fail-fast`; reverted, `git diff --stat` clean, gate `-p moveit-model` green after). |

**Correction, round 17**: that reasoning has a hole, found while biting
`:3003`/`:3038` (old `:2986`/`:3021`) this round — both were originally
placed in this "structurally immune" class. The "any push flips it"
argument only protects against a *false non-empty* result; it says nothing
about whether *staying empty* can itself be produced by more than one
distinct skip guard. `build_group_states` (`robot_model.rs:1598-1636`) has
exactly two: the unknown-group early `continue` (`:1600`) and the
empty-after-filtering skip (`:1629`) — neither ever pushes, so the "any
push" argument never engages, and the two causes genuinely funnel into the
same `is_empty() == true` outcome. The isolating mutation (neutralizing
`:1629`) proved they are separable, which only holds because a real, in-fence
bite happened to be run on this pair — nothing in the class-level argument
would have caught the gap on its own. `:3003`/`:3038` are excluded from the
list above for this reason. Not re-audited this round: whether any of the
remaining 11 sites has the same "more than one skip path funneling into one
empty result" shape rather than a genuinely single cause. Two of them
(`:2411`, `:2518`, the mesh-skip tests) looked like candidates on inspection
but are safe for a *different* reason — each carries its own sibling
`detail`-string assertion in the same test (`:2472`-class, already
`distinguishable payload`) that independently proves which of the four mesh
failure branches fired, so their `is_empty()` is a secondary confirmation,
not the only evidence. The other 9 were not re-checked against this
specific failure mode; flagging rather than asserting they are clean.

### Result

2 funnels found in the 53-site fence; both had an untested cause, both
fixed with a new isolating-mutation-proven test; all previously-bitten
causes on both funnels re-verified this round rather than carried forward
from prior reports. 0 additional blind sites found — every already-verdicted
`discriminating` site outside these two funnels is either single-guard or
carries a distinguishing payload, named above.

### Commands run (round 15)

- `python3 tools/ci/count-coarse-assertions.py <six paths>` — 53, reconciled against round 14's +1
- `cargo nextest run -p moveit-planning check_start_state_bounds --no-fail-fast` — 8 isolating-mutation runs (5 causes × new-test-passes-clean check, each reverted)
- `cargo nextest run -p moveit-model visual_mesh --no-fail-fast` — 1 isolating-mutation run, reverted
- `git diff --stat` after all reverts — only test modules and the two new tests' bodies changed; both subject functions byte-identical to pre-round
- `cargo fmt --all`
- `cargo clippy -p moveit-model --all-targets -- -D warnings` — clean
- `cargo clippy -p moveit-planning --all-targets -- -D warnings` — clean
- `cargo nextest run -p moveit-model` — 137 tests run, 137 passed, 0 skipped
- `cargo nextest run -p moveit-planning` — 43 tests run, 43 passed, 0 skipped

Gate scope: `-p moveit-model -p moveit-planning`, source-touching round, all
steps run and clean as owed. `moveit-constraints` (`decide.rs`) was audited
but not touched — its two candidate sites (`:1161,1166`) are single-guard,
excluded above; no fix owed, no gate owed for that crate this round.

## Round 16 — the orphan audit: re-derived independently, corrected against two user retractions mid-turn

The user's brief assigned 42 (`moveit-model` 39 + `moveit-planning` 3, plus a
share of `moveit-constraints/tests/decide.rs`), against a stated 694-site
corpus. Two retractions arrived mid-round, both honored before finishing:
first, 706 (not 694), then a full withdrawal of 706 itself — the scratchpad
reconciliation script's first-column regex silently dropped ledger rows with
trailing annotation text, undercounting every ledger's matched rows and
inflating every orphan count. The reproducible figure, from `main`'s newly
committed `tools/ci/reconcile-assertion-ledgers.py`, is **697 = 469 + 228**,
and this round's corrected assignment is **29**, not 42.

**Did not take any of the four numbers on the user's word.** Ran
`python3 tools/ci/count-coarse-assertions.py crates/moveit-model
crates/moveit-planning crates/moveit-constraints/tests/decide.rs` myself —
84 real sites (excluding one `helper_body` line), splitting 60/10/14 across
the three paths. That 60/10 split matches the user's independently-derived
per-crate totals exactly, confirming the fresh scan and my prior round's
work are not drifting apart. `git merge main` (fast-forward to `919c982`,
pulling in `tools/ci/reconcile-assertion-ledgers.py`,
`tools/ci/assertion-ledger-equivalences.json`, and
`doc/assertion-discrimination-orphans.txt`), then ran the committed
instrument rather than a hand-rolled diff: `matched + orphans == scanner
sites` self-check passed (476 + 222... — the tool's live run differs by one
scanner site from the committed 228/697 snapshot, both measured against a
moving `main`; not chased further since the committed
`doc/assertion-discrimination-orphans.txt` is what the user pinned my
assignment against). Filtered `doc/assertion-discrimination-orphans.txt` to
my three paths: **29 sites, exact match to the corrected assignment.** No
adjustment made to reach this number — it is what both independent
instruments (my fresh scan minus my own ledger's citations, and the
committed reconciliation tool) agree on.

### The false-orphan trap, found in my own ledger

The reconciliation tool's `unresolved_citations` diagnostic (window ±5
lines, everything past that reported not guessed) flagged exactly the stale
citations I had already found independently by direct re-reading:
`robot_model.rs:2764,2885,2886,2986,3021,3031,3376` and
`check_start_state_bounds.rs:302,328,377`, plus `mesh_search_paths.rs:130`
(a comment line, not the assertion it once was). Every one of these is a
site my ledger already verdicted, at a line number code growth since
Round 13/14/15 has moved — new tests inserted earlier in `robot_model.rs`
(the mimic-diagnostic cluster ~1965-2110, the mesh-failure cluster
~2480-2600, the expanded end-effector cluster ~2740-2905) push everything
after them forward by 17-25 lines depending on position. Confirmed
byte-identical content at each new location by direct read before treating
any of them as a pure rename rather than a real gap.

### 29-site classification

**A — stale citation only, content and verdict unchanged (11 sites, 8 rows in the table below)**

| current file:line | old citation | verdict (unchanged) | correction |
|---|---|---|---|
| `mesh_search_paths.rs:162` | `:139,140` | discriminating | line only — Round 14's isolating mutation on `:86` (`strip_prefix`) unchanged |
| `robot_model.rs:2822` (`end_effector_wires_name_and_falls_back_to_fewest_joints_parent`) | `:2764` | discriminating | line only — Round 13's cross-test sibling reasoning (`:2755,2762` non-empty) unchanged |
| `robot_model.rs:2943` | `:2885` | single-branch, excluded | line only — Round 15's field-default reasoning (`set_end_effector_parent` never called in this fixture) unchanged |
| `robot_model.rs:3089` (`variable_default_positions_returns_none_for_unknown_state_name`) | `:3031` | not-this-family | line only — Round 13's vacuous reasoning (`group_state_test_srdf("")`, zero elements) unchanged |
| `robot_model.rs:3468,3469,3470,3471` | `:3376,3377,3378,3379` | **funnel, closed round 18** (was single-branch) | round 16: line only, confirmed byte-identical body by direct read. Round 17: line renumbered to `:3393-3396` (missed in this table, caught round 18's CORRECTION 2). Round 18: renumbered again to `:3427-3430` by this round's own fixture edit, and the "single-branch" verdict itself overturned — see the round-18 table above for the isolating-mutation evidence and the new-test closure. |
| `check_start_state_bounds.rs:358` (`an_out_of_bounds_velocity_is_rejected`) | `:302` | discriminating | line only — Round 13/15's isolating mutation (position guard) unchanged |
| `check_start_state_bounds.rs:384` | `:328` | discriminating | line only — Round 13/15's isolating mutation (velocity guard) unchanged |
| `check_start_state_bounds.rs:433` | `:377` | not-this-family | line only — Round 13's range-check reasoning unchanged |

**A2 — stale citation plus a genuine correction (3 sites)**

| current file:line | old citation | old verdict | corrected verdict | why |
|---|---|---|---|---|
| `robot_model.rs:2944` | `:2886` | discriminating (Round 13, cross-test sibling `:2755,2762`) | **not-this-family** | Round 13's own evidence compared this test's result against a *different* test's fixture (one with an `<end_effector>` element), not this test's own causal chain — exactly the reading-vs-mutation gap Round 14 already censured. This test's SRDF (`end_effector_test_srdf("")`) declares zero end effectors at all, so `attached_end_effector_names()` has nothing to ever populate for *any* group, for the identical field-default reason Round 15 already applied one line above at `:2902` (old `:2885`). Reclassified to match, not left standing on a cross-fixture comparison |
| `robot_model.rs:3044` | `:2986` | discriminating (Round 13, reading: "distinct decision from `:3021`'s") | discriminating, **now bitten** | Round 13's evidence was reading alone. Isolating mutation this round on `build_group_states` (`robot_model.rs:1598-1636`): neutralized the empty-state skip (`:1629`, `if !state.is_empty()` → `if true`) — `robot_model.rs:3038` (`group_state_naming_an_unknown_group_is_ignored`)'s test (all-values-unusable) failed, this site's test (`group_state_naming_an_unknown_group_is_ignored`, the unknown-group skip at `:1600`) stayed green. Confirms the two guards are genuinely separable, not just plausibly so |
| `robot_model.rs:3079` | `:3021` | discriminating (Round 13, reading) | discriminating, **now bitten** | Sibling of the above — see the same mutation. Neutralizing `:1629` failed exactly this test (`arm.default_state_names().is_empty()` becomes false, an empty state gets inserted). Reverted; `git status --short` clean after |

**B — genuinely new sites, already covered by a prior round's bite but never given a formal `file:line` row (4 sites)**

| file:line | verdict | evidence |
|---|---|---|
| `mesh_search_paths.rs:179` | discriminating | Round 14's `malformed_package_uri_with_no_relative_path_does_not_resolve`, isolating mutation on `:87` (`split_once('/')?`), narratively described but never tabled |
| `mesh_search_paths.rs:192` | discriminating | Round 14's `missing_file_does_not_resolve`, isolating mutation on `:89` (`candidate.is_file()`), same gap |
| `robot_model.rs:2734` | discriminating | Round 15's Funnel 2 fix, `mesh_with_an_empty_filename_leaves_visual_mesh_filename_none`, isolating mutation on `!filename.is_empty()`, narratively described (Round 15's own text) but never tabled |
| `check_start_state_bounds.rs:474` | not-this-family | Round 15's new `a_planar_joint_past_pi_is_wrapped_...` test's own `(-PI..=PI).contains(&wrapped)` line — same numeric-range-on-computed-result exclusion as `:433`, never tabled |

**C — genuinely new, triaged fresh this round (11 sites)**

| file:line | verdict | evidence |
|---|---|---|
| `robot_model.rs:2093,2094,2095,2096` | discriminating, via same-test sibling payload | `mimic_mutual_cycle_clears_every_mimic_in_the_model`'s four `mimic().is_none()` checks are secondary confirmations; the preceding `:2051` (`assert!(matches!(model.diagnostics(), [Diagnostic::MimicCycle]))`) already names which of the three mimic-clearing causes (`MimicUnknownJoint`, `MimicDofMismatch`, `MimicCycle`) fired via a distinguishable enum-variant payload, the same exclusion class as `robot_model_parity.rs:356`'s classification-tag comparison. Re-read `resolve_mimic` (`robot_model.rs:1355-1423`) this round to confirm the three diagnostics are still pushed at three structurally separate sites (`:1369`, `:1376`, `:1413`), matching the in-source doc comment's round-8 claim |
| `robot_model.rs:2151` | discriminating, via same-test sibling payload | `mimic_with_mismatched_dof_is_dropped_with_a_diagnostic`'s `mimic().is_none()` is preceded by `assert_eq!(model.diagnostics(), [Diagnostic::MimicDofMismatch{..}])` in the same test — same exclusion as above |
| `robot_model.rs:2533` | not-this-family | `std::fs::read(&path).is_err()`, message reads "precondition: this test needs {path} to be unreadable" — a check on the test's own fixture setup, not on `RobotModel`, clause 3 |
| `robot_model.rs:2605` | not-this-family | `matches!(shapes[0].shape, Shape::Mesh(_))` — computed classification tag, same exclusion as `robot_model_parity.rs:356` |
| `robot_model.rs:2837` | discriminating | **isolating mutation, run this round**: `get_end_effector` (`:675-686`) folds two causes into one `Error::unknown_name` construction site — `self.groups.get(name)` missing entirely vs. present-but-`.filter(is_end_effector)`-rejected. Neutralized the filter (`.filter(\|_group\| true)`) — `end_effector_wires_name_and_falls_back_to_fewest_joints_parent`'s `model.get_end_effector("arm").is_err()` (`:2837`) failed, `get_end_effector_unknown_name_is_an_error`'s test (`:2856`) stayed green. Reverted; `git status --short` clean |
| `robot_model.rs:2856` | discriminating | mirror of `:2837` — same mutation, this test's assertion (name not a group at all) stayed green because `.get(name)` already returns `None` before the mutated filter ever runs |
| `robot_model.rs:3080` | discriminating | `group_state_where_every_joint_value_is_unusable_stores_no_state_at_all`'s `variable_default_positions("empty").is_none()` — same test, same bitten cause as `:3038` |
| `robot_model.rs:3090` | not-this-family | `variable_default_positions_returns_none_for_unknown_state_name`, `group_state_test_srdf("")` — zero `<group_state>` elements at all, vacuous, same class as `:3048` |

### `decide.rs:183/184` — not orphaned, already covered outside this ledger

Not in the 29-site list (confirmed by direct check of
`doc/assertion-discrimination-orphans.txt`), and I read why: p1-fixtures'
ledger (`doc/assertion-discrimination-ledger-p1-fixtures.md:344-345`)
already bit this exact site — `JointConstraint::new`'s tolerance guard
(`moveit-constraints/src/joint.rs:120`, owned by p1-fixtures per
`doc/folded-operand-guards.md`) — neutralizing `tolerance_above`'s clause
alone fails `decide.rs:183`, `tolerance_below`'s alone fails `:184`, each
leaving the other green. Correctly so: the guard lives in
`moveit-constraints/src/joint.rs`, which is outside this round's fence even
though `decide.rs` (the test file) is nominally mine — biting it would have
required editing a file I do not own. No action taken, none owed.

### Result: 29/29 real orphans closed — 0 blind sites, 1 verdict corrected, 2 sites freshly bitten, 25 by stale-citation or missing-formal-row correction

No fixes to source were left in the tree — both bites this round (`:2837`/
`:2856`'s `get_end_effector`, `:3003`/`:3038`'s `build_group_states`) were
reverted immediately after confirming isolation. The only correction that
changes a verdict is `robot_model.rs:2903` (discriminating →
not-this-family); everything else is a line-number fix or a formal row for
evidence that already existed. This ledger's own citations are what
produced 25 of the 29 "orphans" in the first place — a real instance of the
false-orphan trap the user warned about, not a hypothetical one.

### Commands run (round 16)

- `git status --short` — clean before starting
- `git merge main` — fast-forward `7014cd6` → `919c982`, pulled in the committed reconciliation instrument
- `python3 tools/ci/count-coarse-assertions.py crates/moveit-model crates/moveit-planning crates/moveit-constraints/tests/decide.rs` — 85 raw lines, 84 real sites (1 `helper_body` excluded), 60/10/14 split
- `python3 tools/ci/reconcile-assertion-ledgers.py` — 698 sites, 476 matched, 222 orphans this round's live `main`, self-check `True`
- `grep -c` on `doc/assertion-discrimination-orphans.txt` filtered to my three paths — 29
- Isolating mutation, `get_end_effector`'s filter (`robot_model.rs:684`): `cargo nextest run -p moveit-model end_effector` — 1/7 failed (`:2837`'s test), reverted, re-ran clean (7/7)
- Isolating mutation, `build_group_states`'s empty-state skip (`robot_model.rs:1629`): `cargo nextest run -p moveit-model group_state` — 1/6 failed (`:3038`'s test), reverted, re-ran clean (6/6)
- Direct read: `resolve_mimic` (`robot_model.rs:1355-1423`), `robot_model.rs:1960-2130,2660-2920,2960-3060,3380-3397`, `mesh_search_paths.rs` (full file), `check_start_state_bounds.rs` (already read prior round), `robot_model_parity.rs` citations spot-checked against the fresh scan (no drift found)
- `cargo fmt --all` — clean
- `cargo clippy -p moveit-model --all-targets -- -D warnings` — clean
- `cargo clippy -p moveit-planning --all-targets -- -D warnings` — clean
- `cargo nextest run -p moveit-model` — 137 tests run, 137 passed, 0 skipped
- `cargo nextest run -p moveit-planning` — 43 tests run, 43 passed, 0 skipped
- `git diff --stat` — only this ledger file changed; both crates' source is byte-identical to pre-round

Gate scope: `-p moveit-model -p moveit-planning`. Doc-only round (0 source
fixes — both mutations reverted after use), so clippy/nextest above were
exercised by the mutation-verification runs, not as a post-fix gate; run
anyway to confirm the reverts left no residue.

## Round 19 — `matches!`-shaped sites in this fence: completeness and the "name the exception" audit

`assert!(matches!(...))` cannot in general say which of several failure
causes produced its input the way an `.is_empty()`/bare-`None` check
sometimes can be shown to (round 18's `robot_model.rs:3427-3430` (`group_of_only_fixed_joints_has_no_joint_roots_or_updated_links`) funnel is
exactly that other family) — but a `matches!` call has its own, sharper
failure mode: the pattern itself may only test *one* discriminant while
several code paths could reach a value that matches it, or the pattern's
payload may be too coarse (e.g. an ignored `_` field) to say which of two
similarly-shaped variants actually landed. **Default verdict for any
`matches!` site in this family is "cannot discriminate"; every
"discriminating" row below states, specifically, what the pattern names
that a bare `bool` comparison would not.**

Completeness first, mechanically, per the user's standing instruction that
`tools/ci/verify-orphan-enumeration.sh` is the authority: run this round,
`OK ... 0 orphans` at commit `7572123` (740 live scanner sites, up from
round 16's 698 — `main` has grown, no orphan gap opened by that growth).
That means every scanner-caught site in this fence — `matches`-kind
included — already has *some* ledger row covering it; the audit below is
about whether that row's verdict is *sound*, not about finding gaps
`verify-orphan-enumeration.sh` missed.

Fresh enumeration, `count-coarse-assertions.py`'s own `matches`-kind
output restricted to this fence's three paths — 8 sites, not the 5+2=7
the census's per-crate table alone would suggest, because `decide.rs`
contributes one the census's crate-level total folds into
`moveit-constraints`' own row:

| site | owning ledger | verdict | exception named? |
|---|---|---|---|
| `moveit-model/src/joint/urdf.rs:366,372,378` (`fixed_floating_and_planar_produce_the_matching_kind`) | p9-ros | not-this-family | n/a — census §9 clause 1, computed success-path `JointKind` tag after `.unwrap()`, not a failure-branch check at all. Read `doc/assertion-discrimination-ledger-p9-ros.md:135-137`: sound, same shape as this ledger's own `robot_model.rs:2605` (`mesh_collision_resolving_to_a_valid_stl_file_builds_a_mesh_shape`) below. Textually inside my fence (`moveit-model/src`) but the guard is exercised by a test file p9-ros already owns per its own fence note; not duplicated here. |
| `moveit-constraints/tests/decide.rs:210` | p1-fixtures | (their verdict, not re-derived here) | n/a — `JointConstraint::new`'s tolerance guard lives in `moveit-constraints/src/joint.rs`, owned by p1-fixtures per `doc/folded-operand-guards.md` (already the precedent this ledger recorded at the `decide.rs:183/184` entry above); `decide.rs` is nominally mine as a test file but the guard is not, so this site is theirs to classify even though it is textually in "my" slice. |
| `robot_model.rs:2092` | this ledger (line ~861, ~1327 above) + independently, p9-ros (`doc/assertion-discrimination-ledger-p9-ros.md:140`) | **discriminating** | **yes, named twice, independently.** This ledger: the pattern is `[Diagnostic::MimicCycle]`, which requires both slice length exactly 1 *and* the one named variant among three the same function can push (`MimicUnknownJoint`, `MimicDofMismatch`, `MimicCycle`) — a bare `bool` (`!diagnostics.is_empty()`) would not tell these apart, but this pattern's payload does. p9-ros's row states the identical reason independently ("requires both exact length 1 and the named variant"), and ran its own bite. Two independent panels landing on the same exception is stronger than either alone. |
| `robot_model.rs:2605` | this ledger (line ~1330 above) | not-this-family | n/a — `matches!(shapes[0].shape, Shape::Mesh(_))` is a computed classification tag on an already-successful build (same exclusion as `robot_model_parity.rs:356` and the `urdf.rs` trio above), not a check that could be blind to which of several failure causes fired. |
| `moveit-planning/src/pipeline.rs:779` (`zero_planners_is_an_error`) | this ledger (line ~120 above) | single-branch | yes — `PipelineError::NoPlanners` is a nullary variant (no payload to lose) with exactly one construction site in the crate (`moveit-planning/src/pipeline.rs:432`, in `generate_plan`, confirmed by `rg 'NoPlanners'` after `10f571f`: still one `return Err` hit); `matches!` cannot be blinder than `==` when there is nothing else the value could be and nowhere else it could come from. |
| `moveit-planning/src/response_adapters/add_time_optimal_parameterization.rs:368` (`adapt_rejects_an_invalid_resample_dt_deferred_from_new`) | this ledger (line ~123 above) | single-branch | yes — same shape: `ResponseAdapterError::Failed{..}` has exactly one construction site (`rg` this round: still one hit, line 139), so the ignored `{..}` payload loses nothing there was ever more than one of. |

Re-ran `rg -n 'PipelineError::NoPlanners' crates/moveit-planning/src/pipeline.rs`
and `rg -n 'ResponseAdapterError::Failed' crates/moveit-planning/src/response_adapters/add_time_optimal_parameterization.rs`
this round rather than trust the two existing rows' construction-site
counts unverified — both still exactly one non-test hit each, so
"single-branch" still holds; no drift since those rows were written.

**Bare-anchor (`is_err`/`is_none`) completeness**, same fence: the census's
own `rg` grammar gives `moveit-model` 17 (was 15 at the last per-crate
census measurement — growth, not a gap: `main` added code, not orphans),
`moveit-planning` 2 (unchanged), `decide.rs` 2. All of these are a subset
of the same 740-site corpus `verify-orphan-enumeration.sh` just confirmed
has zero orphans, and rounds 9–16 above already carry per-site verdicts
(with bites, where a same-test sibling alone would not do) for the
`is_err`/`is_none`/`is_empty`/`eq_none` sites in this fence — no new bare
site appeared in this fence's 15→17 growth that isn't already one of the
698→740 sites the reconciliation tool matched. Not re-walked line by line
again this round: the mechanical gate is the authority the user named for
completeness, and it is clean.

### Commands run (round 19)

- `git status --short` — clean before starting; `git log` confirms `HEAD`
  already at `7572123` (fast-forwarded by the user, no merge needed this
  round)
- `bash tools/ci/verify-orphan-enumeration.sh` — `OK`, 0 orphans, 740 live
  sites, matches committed `doc/assertion-discrimination-orphans.txt`
- `python3 tools/ci/count-coarse-assertions.py crates/moveit-model crates/moveit-planning crates/moveit-constraints/tests/decide.rs | grep ':matches:'` — 8 sites, cross-checked against every row above
- `rg -n 'PositionConstraint::new\('` on each of `decide.rs`/`utils_parity.rs`/`constraint_sampler_manager.rs`/`ik_sampler.rs` — 12/2/6/13 = 33 (see round 10's addendum above)
- `rg -n 'PipelineError::NoPlanners'` and `rg -n 'ResponseAdapterError::Failed'` on their respective files — one non-test construction site each, confirmed still true
- `cargo fmt --all` — clean (no source touched this round)
- `cargo nextest run -p moveit-constraints` — 103 tests run, 103 passed, 1 skipped (pre-existing skip, unrelated)

Gate scope: `-p moveit-constraints` (doc-only round touching that crate's
test-file evidence; `moveit-model`/`moveit-planning` untouched this round,
their round-16/18 gates stand). No source fixes this round — every site
audited was already correctly classified; the audit's output is the
verification itself, not a correction.

## Round 20 — the 8 `moveit-planning` orphans `10f571f` opened, each bitten

`10f571f` (p11-startstate) replaced `PlanningRequest::start_state`'s
transcribed value-plus-`is_diff` pair with a `StartState` sum type, adding
`crates/moveit-planning/src/start_state.rs` outright and shifting
`crates/moveit-planning/src/pipeline.rs`. That created 8 scanner sites in
this ledger's fence with no accounting row — 6 new assertions in the new
file, 2 new ones in `pipeline.rs` — plus one stale citation of this
ledger's own, fixed separately in `9515aa4` — which re-derived both line
numbers in the round-19 `NoPlanners` row above, where they now stand
verified. Neither the pre-`10f571f` numbers that fix replaced nor the ones
it wrote are repeated here: a dead line number in backticks reads as a
live citation and is false by construction, and a live one restated away
from its row is a second copy to drift. Round 19's "the mechanical gate is the authority and it
is clean" therefore describes a tree 68 commits of `main` behind this
round's merge base; this round re-runs it and closes this ledger's half
of what it now reports.

**Every row below is a fresh isolating mutation run this round**, not a
reading. Eleven mutations, each applied alone to a byte-identical tree and
reverted immediately; `cargo nextest run -p moveit-planning
--no-fail-fast` after each, so the blast radius is the full 57-test count
and not a fail-fast prefix. Baseline before and after: `57 tests run: 57
passed, 0 skipped`, `git diff --stat` empty.

### Family membership (census §9), stated before the verdicts

All 8 pass the three clauses. Clause 1: the inspected value is a
`Result::Err` (the six `start_state.rs` sites, via `assert_err_mentions`'s
`expect_err`), a `PipelineError` payload (`pipeline.rs:1184`), or an
absence signal for "no adapter ever observed the scene"
(`pipeline.rs:1191`) — none is an informative success-path value. Clause
2: each is produced by a written guard or `?`-propagation —
`start_state.rs:143`, `:228`, `:234`, `:243`, `:187`, `:195` and
`pipeline.rs:441` — every one of which an engineer could have written
backwards. Clause 3: each decision belongs to the function the test's own
name calls its subject (`StartState::new`, `StartStateOverride::new`,
`StartState::apply_to`, `generate_plan`).

### The subject's guards, enumerated before mutating them

`StartState::new` (`crates/moveit-planning/src/start_state.rs:137-157`)
has one guard reachable only under `names.is_empty()` (`:138`):
`!positions.is_empty() || !velocities.is_empty()` (`:143`) — one
construction site folding **two** named operands, so two branches per
`doc/folded-operand-guards.md`, mutated per operand below.
`StartStateOverride::new` (`:227-258`) has three: `names.is_empty()`
(`:228`), `positions.len() != names.len()` (`:234`), and
`!velocities.is_empty() && velocities.len() != names.len()` (`:243`,
again two operands, again mutated separately).
`StartState::apply_to` (`:173-198`) has two `?` sites: the position write
wrapped with index, value and name (`:183-187`) and the velocity write
propagated bare (`:195`).

Both writes fail for exactly one cause each — `RobotState::set_variable_position`
(`crates/moveit-state/src/state.rs:557-564`) and `set_variable_velocity`
(`crates/moveit-state/src/state.rs:401-406`) each resolve the name through
`variable_index(name)?` and have no other fallible step. Since the position
write resolves the same name first and returns on failure, `:195` is
unreachable for the unknown-name cause — which is what the in-source
comment at `:191-194` claims, confirmed here by reading both callees
rather than taking the comment's word.

### Per-site verdicts — `crates/moveit-planning/src/start_state.rs`

| file:line | enclosing test (verified by content) | verdict | evidence |
|---|---|---|---|
| `crates/moveit-planning/src/start_state.rs:322` | `an_empty_override_is_unconstructible_through_the_override_constructor_too` | discriminating | **bite**: `names.is_empty()` (`:228`) → `false`. Exactly this test failed (`57 tests run: 56 passed, 1 failed`); the other five sites below stayed green, including `:330`/`:341`, whose guard shares the phrase this test needles. Neutralizing `:228` cannot be masked by the other two override guards — for `(vec![], vec![], vec![])`, `positions.len() == names.len() == 0` and `velocities` is empty, so both `:234` and `:243` are false and the call returns `Ok`. |
| `crates/moveit-planning/src/start_state.rs:330` | `positions_without_names_is_rejected_not_read_as_current_state` | discriminating | **bite, per operand**: `!positions.is_empty()` (left operand of `:143`) → `false`. Exactly this test failed; `:341`'s stayed green. The needle also names the counts (`"…but carries 1 position(s) and 0 velocity(ies)"`), so the message itself says which operand fired. |
| `crates/moveit-planning/src/start_state.rs:341` | `velocities_without_names_is_rejected_not_read_as_current_state` | discriminating | **bite, the mirror**: `!velocities.is_empty()` (right operand of `:143`) → `false`. Exactly this test failed; `:330`'s stayed green. Both operands of the one construction site are therefore covered, which a single whole-condition mutation would not have shown. |
| `crates/moveit-planning/src/start_state.rs:352` | `a_name_without_a_position_is_rejected_not_read_as_a_velocity_only_overlay` | discriminating | **bite**: `positions.len() != names.len()` (`:234`) → `false`. Exactly this test failed. Note what the mutation exposes downstream — with `:234` neutralized, `apply_to`'s `over.positions()[index]` (`:182`) indexes past the end, so this guard is the only thing standing between a short `position` array and a panic. |
| `crates/moveit-planning/src/start_state.rs:360` | `a_short_velocity_array_is_rejected_rather_than_read_past_its_end` | discriminating | **bite, per operand**: `velocities.len() != names.len()` (right operand of `:243`) → `false`. Exactly this test failed. The left operand (`!velocities.is_empty()`, the "no velocities at all is legal" escape) is covered too, by its own mutation → `true`: 5 tests failed (`a_start_state_naming_an_unknown_variable_fails_before_any_adapter_or_planner_runs`, `the_requested_start_state_reaches_the_scene_before_the_request_adapters_run`, `an_overlay_pairs_each_value_with_its_own_name`, `an_overlay_writes_the_named_variables_and_leaves_the_rest_at_the_current_state`, `an_overlay_naming_a_variable_the_model_lacks_is_rejected_at_apply_time`), so neither operand is blind. |
| `crates/moveit-planning/src/start_state.rs:469` | `an_overlay_naming_a_variable_the_model_lacks_is_rejected_at_apply_time` | discriminating | **two bites, both isolating.** (a) Drop the wrap at `:183-187` (`state.set_variable_position(name, position)?`): exactly this test failed — and `crates/moveit-planning/src/pipeline.rs:1184`, which needles only `"no_such_joint"`, stayed green, so the two sites are provably not testing the same thing. (b) Pair every name with `over.positions()[0]` (`:182`): this test failed on the value component (`position 0.2` no longer appears), alongside `an_overlay_pairs_each_value_with_its_own_name`. The needle's index and value are therefore both live, which is what the test's own comment claims they are for. |

**Fragility flagged, not fixed** (same treatment as round 11's three
fragile-but-currently-unique needles): `:322`'s needle `"names no
variable"` is a substring of *two* messages — `:228`'s
(`"start_state.joint_state override names no variable; …"`) and `:143`'s
(`"start_state.joint_state names no variable but carries …"`). It
discriminates today only because `:143` lives in `StartState::new` and
this test calls `StartStateOverride::new` directly, so `:143` is
unreachable from it — confirmed by the bites above (mutating either
operand of `:143` left this test green). A future refactor that routed
the empty case through `StartStateOverride::new` would silently make the
needle ambiguous with no test failing. Widening the needle to
`"override names no variable"` would close it; not done here, since it is
a test-source change outside the row this round owes.

### Per-site verdicts — `crates/moveit-planning/src/pipeline.rs`

Both sites are in the same test and the two mutations below separate them
in both directions.

| file:line | enclosing test (verified by content) | verdict | evidence |
|---|---|---|---|
| `crates/moveit-planning/src/pipeline.rs:1184` | `a_start_state_naming_an_unknown_variable_fails_before_any_adapter_or_planner_runs` | discriminating | **bite**: replace `.map_err(PipelineError::StartState)` (`crates/moveit-planning/src/pipeline.rs:441`) with a closure discarding the source and constructing a fixed message. The panic names this exact line — `panicked at crates/moveit-planning/src/pipeline.rs:1104:17: the rejection must name the variable that could not be written, got: construction failed: rejected` — and `:1191` did not fire, so this assertion alone carries the "the message survives the variant wrap" claim. The variant itself has exactly one construction site (`:441`, in `generate_plan`), so the outer `match` arm's job is narrower than this needle's. |
| `crates/moveit-planning/src/pipeline.rs:1191` | `a_start_state_naming_an_unknown_variable_fails_before_any_adapter_or_planner_runs` | discriminating | **bite**: move the `apply_to` call (`crates/moveit-planning/src/pipeline.rs:438-441`) *after* `run_request_adapters` (`:443`). The panic names this exact line — `panicked at crates/moveit-planning/src/pipeline.rs:1111:9: Semantic 7 applies the start state before request_chain, …` — while `:1184` stayed green (the error is still a `StartState` variant carrying the same message). The sibling ordering test at `crates/moveit-planning/src/pipeline.rs:1121-1162` also failed, as that mutation's direct target. |

`:1191` is a `Vec::is_empty()` site, so it gets round 18's sharper
standard rather than the retired "any push flips it" argument: enumerate
every **subject-side** path that leaves `seen` empty. There are exactly
two. The `NoPlanners` early return (`crates/moveit-planning/src/pipeline.rs:431-433`)
returns before any adapter runs — ruled out inside this same test by the
preceding `match err { PipelineError::StartState(e) => … , other =>
panic!(…) }` (`crates/moveit-planning/src/pipeline.rs:1182-1189`), a
same-test sibling that names the variant. The `?` at
`crates/moveit-planning/src/pipeline.rs:441` is the claimed cause. There is
no third: `run_request_adapters` (`crates/moveit-planning/src/lib.rs:447-457`)
is an unconditional `for adapter in chain` loop with no skip guard —
read this round, not assumed.

### Result: 8/8 orphans closed with rows, 0 blind sites, 0 source changes

Orphan count over this round, from `python3
tools/ci/reconcile-assertion-ledgers.py`: 47 orphans + 8 unresolved at the
merge base, 47 + 7 after `9515aa4`'s citation fix, 39 + 7 after this
section. The 39 remaining then, and the 7 remaining unresolved, were all
p9-ros's — live in another worktree this round and deliberately untouched
here.

Re-measured after this section merged (`f728b15`, which also carries
p9-ros's own half): **32 orphans, 0 unresolved**, none in
`crates/moveit-planning`. The 32 are `ros/moveit-ros/src/scene/planning_scene.rs`
19, `ros/moveit-ros/src/planning.rs` 10,
`ros/moveit-ros/src/constraints/orientation.rs` 2 and
`ros/moveit-ros/src/scene/shapes.rs` 1 — still p9-ros's, still not this
ledger's to close. Neither of the two gates that report them
(`tools/ci/verify-orphan-enumeration.sh`,
`tools/ci/check-citation-drift.py`) runs in CI: `.github/workflows/ci.yml`'s
"ci checks" step globs `tools/ci/check-*.sh`, which matches neither a
`verify-`-prefixed script nor a `.py`. `tools/ci/verify-clean-checkout.sh`
therefore exits `0` — truthfully, it runs the ci.yml steps — while both of
these are red. Reported here rather than fixed: wiring them in turns `main`
red for every panel at once, which is the merger's call, not a side effect
of an orphan round.

`doc/assertion-discrimination-orphans.txt` is **not** regenerated. Its
header still reads 770 sites / 0 orphans against source commit
`0b4e79b5`, which is stale in both directions (804 live sites, 39 live
orphans); regenerating it now would write p9-ros's 39 open orphans into
the expected set — exactly the laundering
`reconcile-assertion-ledgers.py`'s own header warns about — so it is left
for whoever closes the last of them.

`doc/citation-classes.txt` is **not** re-frozen either, at the
orchestrator's instruction that they own it at merge. What this round
adds to that diff, measured rather than assumed — `check-citation-drift.py`
run against `eae7de8` with this ledger restored to its merge-tip bytes
exits `0`, so every entry below is this round's: **0 demoted, 0
recounted, 0 promoted, 24 undeclared, 2 retired**. 22 of the undeclared
are this section's own citations entering the corpus; the other two are
the corrected line numbers `9515aa4` wrote into the round-19 `NoPlanners`
row. The 2 retired are not 0, so the "leave it alone if nothing retires"
condition does not literally hold: both are that same row's
pre-`10f571f` numbers, retired by the fix that replaced them rather than
by a citation going unchecked. Substantive classes are clean — `0 out-of-bounds, 0
anchor-mismatch, 0 unresolvable` — and two anchor-mismatches this
section introduced on first write (a row anchoring
`crates/moveit-planning/src/pipeline.rs:1191` to the sibling ordering
test rather than its own, and a prose bullet whose nearest preceding name
was `set_variable_velocity` when the citation was
`set_variable_position`'s) were found by the gate and fixed before
commit, not declared.

### Commands run (round 20)

- `git merge main` — fast-forward, 68 commits, tip `eae7de8`; run before deriving any line number in this section
- `python3 tools/ci/reconcile-assertion-ledgers.py --emit-orphans` — 8 of the 47 orphans in this ledger's fence, enumerated by path
- Content-verification of all 8 citations before writing them: a brace-depth walk printing each cited line's innermost enclosing `fn` — all 8 resolve to the test each row names
- Read `set_variable_position` (`crates/moveit-state/src/state.rs:557-564`), `set_variable_velocity` (`crates/moveit-state/src/state.rs:401-406`) and `run_request_adapters` (`crates/moveit-planning/src/lib.rs:447-457`) rather than trusting the comments that describe them
- 11 isolating mutations, each alone, each followed by `cargo nextest run -p moveit-planning --no-fail-fast` and an immediate revert from a pre-round copy: `start_state.rs:143` left operand, `:143` right operand, `:228`, `:234`, `:243` whole condition, `:243` left operand, `:243` right operand, `:183-187` wrap, `:182` index, plus `pipeline.rs:441` map_err and the `pipeline.rs:438-443` reorder
- `cargo nextest run -p moveit-planning --no-fail-fast` — baseline and post-revert both `57 tests run: 57 passed, 0 skipped`; `git status --short` and `git diff --stat` empty after the last revert

Gate scope: `-p moveit-planning`. Doc-only round — every mutation was
reverted and no source file differs from its pre-round bytes — so
clippy/nextest below were exercised by the mutation runs themselves; run
again anyway to confirm the reverts left no residue.

## Re-anchored by D8 (planner-type unification)

`PORTING-PLAN.md` D8 merged `moveit_planners_sbp::registry`'s private
`PlanningRequest`/`PlanningResponse` into `moveit-planning`'s and moved
`PlannerManager`/`PlanningContext` there too, which rewrote most of
`registry.rs`'s test module and shifted `pipeline.rs`. Four citations in this
file moved:

* `crates/moveit-planners-sbp/src/registry.rs:1247`
  (`unknown_group_is_rejected_before_any_search_runs`)
* `crates/moveit-planners-sbp/src/registry.rs:1867`
  (`solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable`)
* `crates/moveit-planners-sbp/src/registry.rs:2024`
  (`path_constraints_solver_wiring_matches_the_call_site`)
* `crates/moveit-planning/src/pipeline.rs:779` (`zero_planners_is_an_error`,
  both rows; the `PipelineError::NoPlanners` construction site cited in their
  evidence is now `crates/moveit-planning/src/pipeline.rs:432`)

Only the current location is spelled here. A pre-D8 line number written as
`file.rs:NNN` reads to every reader and to `tools/ci/check-citation-drift.py`
alike as a claim about the tree in front of them, and it is false the moment
it is written; the branch point (`a35bc2e`) names the old tree without
pointing into it.

Each new line was obtained by aligning `git show a35bc2e:<file>` against the
working tree with `difflib` and then reading the row's own named test
function at the result, not by nearest-line proximity.

`crates/moveit-planners-sbp/src/registry.rs:1247` is the one that is more than a
move. D8 made
`moveit_planning::PlanError` a `Box<dyn Error + Send + Sync>`, so
`unknown_group_is_rejected_before_any_search_runs` no longer matches the
concrete variant directly — it downcasts first and then matches. The row's
recorded bite was run against the old shape, so it was re-run against the new
one: replacing `JointModelGroupSpace::new`'s `SbpError::UnknownGroup { .. }`
with `SbpError::NoSubspaces` fails this test **and**
`joint_model_group_space::tests::unknown_group_is_rejected`, with 101 of 112
green; reverted, `git diff --stat` clean. Recorded in full as bite B6 of
`doc/assertion-discrimination-ledger-d8-planner.md`. The verdict
(single-branch) is unchanged.
