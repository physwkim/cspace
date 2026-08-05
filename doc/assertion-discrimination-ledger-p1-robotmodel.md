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
| `time_optimal_trajectory_generation.rs:1030` | `TotgOptions::with_resample_dt` combined guard | `resample_dt_zero_is_rejected_not_hung` | single-branch | commit `52e38a3` (structural: `resample_dt` made `pub(crate)`, settable only through this one validating builder — no other construction path exists) |
| `time_optimal_trajectory_generation.rs:1043` | same | `resample_dt_negative_is_rejected_not_silently_truncated` | single-branch | commit `52e38a3` |
| `time_optimal_trajectory_generation.rs:1112` | same | `resample_dt_nan_is_rejected` | single-branch | commit `52e38a3` |
| `time_optimal_trajectory_generation.rs:1125` | same | `resample_dt_positive_infinity_is_rejected` | single-branch | commit `52e38a3` |
| `time_optimal_trajectory_generation.rs:1136` | same | `resample_dt_negative_infinity_is_rejected` | single-branch | commit `52e38a3` |
| `robot_trajectory.rs:484` | `add_suffix_way_point`'s empty+nonzero-dt guard | `add_suffix_way_point_on_an_empty_trajectory_rejects_a_nonzero_dt` | single-branch | bite: neutralized guard (`if false && ...`) → test failed; reverted |
| `robot_trajectory.rs:532` | `set_way_point_duration_from_previous`'s index-0 guard | `set_way_point_duration_from_previous_at_zero_rejects_a_nonzero_value` | single-branch | bite: neutralized guard → test failed; reverted |
| `robot_trajectory.rs:550` | `append`'s empty+nonzero-dt guard | `append_onto_an_empty_trajectory_rejects_a_nonzero_dt` | single-branch | bite: neutralized guard → test failed; reverted |
| `robot_trajectory.rs:568` | `way_point`'s `.get(index).ok_or_else` | `empty_trajectory_accessors_return_typed_errors_not_panics` | single-branch | bite: forced the `None` arm to panic instead of converting → test failed at this line specifically; reverted |
| `robot_trajectory.rs:569` | `first_way_point`'s `.front().ok_or_else` | same test | single-branch | bite: forced panic on `None`; failed at this line, isolated from `last_way_point`'s sibling; reverted |
| `robot_trajectory.rs:570` | `last_way_point`'s `.back().ok_or_else` | same test | single-branch | bite: forced panic on `None`; failed at this line, isolated from `first_way_point`'s sibling; reverted |
| `robot_trajectory.rs:668` | `way_point`'s guard, out-of-range case | `out_of_range_index_access_is_a_typed_error` | single-branch | same anchor/bite as :568 — same guard, non-empty fixture |
| `robot_trajectory.rs:669` | `way_point_mut`'s `.get_mut(index).ok_or_else` | same test | single-branch | bite: forced panic on `None` → test failed at this line; reverted |
| `robot_trajectory.rs:670` | `remove_way_point`'s explicit `if index >= len` | same test | single-branch | bite: neutralized guard → test failed at this line; reverted |
| `robot_trajectory.rs:749` | `for_group_name`'s `joint_model_group(group)?` | `unknown_group_name_is_a_typed_error_not_a_silent_whole_robot_fallback` | single-branch | bite: forced panic on the `?`'s `Err` arm → test failed; reverted |
| `ruckig_smoothing.rs:199` | `trajectory.group().is_none()` — precondition sanity check on `RobotTrajectory::new`, not on `apply_smoothing`'s guard | `no_group_set_is_an_error` | not-this-family | commit `6a6b46a` (the test's substantive assertion, a few lines below this one, already checks `apply_smoothing`'s error message; this line only confirms the fixture itself has no group set) |

## moveit-planners-chomp (14)

Cost hazard note (per brief): the crate's convergence tests are slow.
Every bite below was scoped to a single `-E 'test(...)'` filter
expression, never the full crate; no bite touched a convergence test.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `cost.rs:424` | `max_quad_cost_inv_value`'s zero-dimension guard | `max_quad_cost_inv_value_rejects_zero_free_points` | single-branch | bite: neutralized guard (`if false`) → test failed (`unwrap_err()` on `Ok`); reverted |
| `cost.rs:493` | `cost()`'s length guard | `cost_and_derivative_reject_mismatched_length` | discriminating | bite: neutralized `cost()`'s guard alone (leaving `derivative()`'s intact) → test failed via nalgebra dimension panic at the `cost()` call specifically; reverted |
| `cost.rs:497` | `derivative()`'s length guard | same test | discriminating | bite (mirror): neutralized `derivative()`'s guard alone (leaving `cost()`'s intact) → test failed via nalgebra panic at the `derivative()` call specifically; reverted |
| `optimizer.rs:2485` | `add_increments_to_trajectory`'s shape guard | `add_increments_to_trajectory_rejects_shape_mismatch` | single-branch | bite: neutralized guard → test failed (nalgebra dimension panic); reverted |
| `optimizer.rs:2540` | `handle_joint_limits`'s length guard | `handle_joint_limits_rejects_joint_costs_length_mismatch` | single-branch | bite: neutralized guard → test failed (`unwrap_err()` on `Ok`); reverted |
| `planner.rs:1063` | `validate_recovery_time_limit`'s combined finite/range guard | `validate_recovery_time_limit_rejects_a_value_whose_plus_five_overflows_i32` | single-branch | bite: forced the guard's condition to `true \|\| ...` → all 4 rejection tests failed, both boundary-accept tests stayed green; reverted |
| `planner.rs:1087` | same | `validate_recovery_time_limit_rejects_a_value_whose_plus_five_underflows_i32` | single-branch | same bite |
| `planner.rs:1092` | same | `validate_recovery_time_limit_rejects_nan` | single-branch | same bite |
| `planner.rs:1097` | same | `validate_recovery_time_limit_rejects_infinity` (+inf) | single-branch | same bite |
| `planner.rs:1098` | same | same test (-inf) | single-branch | same bite |
| `trajectory.rs:695` | `from_num_points`'s `num_points < 2` guard (`Error::Other`, distinct variant from the function's other guard, `Error::UnknownName` via `?`) | `from_num_points_rejects_fewer_than_two_points` | discriminating | bite: neutralized the `num_points < 2` guard → test failed (subtract-with-overflow panic downstream); the `matches!(err, Error::Other(_))` shape excludes the sibling `UnknownName` variant by construction; reverted |
| `trajectory.rs:697` | same guard, `num_points == 0` case | same test | discriminating | same bite/anchor |
| `trajectory.rs:997` | `source.group().is_none()` — precondition sanity check, not `fill_in_from_trajectory`'s own guard | `fill_in_from_trajectory_rejects_a_trajectory_with_no_group` | not-this-family | commit `77a5d7d` (the test's substantive assertion, a few lines below, already checks the error message to discriminate among `fill_in_from_trajectory`'s several `Error::other` sites; this line only confirms the fixture has no group) |
| `utils.rs:194` | `robot_state_to_array`'s two `UnknownName` sites (group vs. joint) | `robot_state_to_array_rejects_an_unknown_group_name_as_a_typed_error` | discriminating | commits `67446b2`/`a90b5dd` (checks `kind == "group"`, discriminating from the joint-name sibling) |

## moveit-planners-sbp (7 — own fence)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `goal_sampler.rs:220` | `sample_goal`'s single fallthrough `None` (budget-exhausted) | `sample_goal_carries_the_previous_draws_result_forward_as_the_next_seed` | single-branch | bite: changed the trailing `None` to `Some(...)` → test failed at this assertion; reverted |
| `goal_sampler.rs:298` | same anchor | `path_constraints_end_to_end_wired_vs_unwired` (unwired-half assertion) | single-branch | same bite/anchor as :220 |
| `nn.rs:227` | `Gnat::nearest`'s `self.root.as_ref()?` empty-index guard | `empty_index_has_no_nearest` | single-branch | bite: forced panic on the `None` arm → test failed; reverted |
| `planning_scene_validity.rs:419` | `PositionConstraint::new`'s two `UnknownName` sites (link_name vs. frame_id) | (world-object-transform test) | discriminating | commit `1397dea` (checks `kind: "frame"`/`name == "table"`; commit message records that swapping the expected kind to the link_name sibling had kept the test passing before the fix) |
| `registry.rs:851` | `JointModelGroupSpace::new`'s single `UnknownGroup` construction site, reached through `get_planning_context` | `unknown_group_is_rejected_before_any_search_runs` | single-branch | bite: forced panic on the guard's `Err` arm → test failed; reverted. `get_planning_context`'s own doc comment states this is its only failure mode |
| `registry.rs:1329` | `PlanningContext::solve`'s single `NoGoalSample` construction site (`sample_goal(...).ok_or(...)`) | `solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable` | single-branch | bite: forced panic on the `None` arm → test failed; reverted |
| `registry.rs:1486` | `resolve_constraint_sampler`'s `None` passthrough from `moveit_constraints::select_default_sampler` | `path_constraints_solver_wiring_matches_the_call_site` | single-branch | bite: forced panic on the wrapper's `None` arm → test failed; reverted. Caveat: `select_default_sampler`'s own internal branching lives in `moveit-constraints`, outside this round's fence — not independently audited beyond confirming this test's fixture always supplies a valid `group_name`, so the sibling "unknown group" `None` inside that function is not reachable from this fixture |

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
| `planner.rs:1052` | `extract_seed_trajectory`'s guard A (empty-input) of 3 `Ok(None)` sites | `extract_seed_trajectory_returns_none_for_zero_constraints` | discriminating | round-report (in-source comment, round 2/§3a): isolating mutation on A alone fails this test and leaves B's/C's tests green — verified against current source (3 `Ok(None)` sites confirmed at their stated positions) |
| `planner.rs:1139` | same function, guard C (per-joint name mismatch) | `extract_seed_trajectory_returns_none_when_joint_name_mismatches_group` | discriminating | same round-report; isolating C alone fails only this test |
| `planner.rs:1164` | same function, guard B (per-waypoint dof-count mismatch) | `extract_seed_trajectory_returns_none_when_dof_mismatches_group` | discriminating | same round-report; isolating B alone fails only this test |
| `planner.rs:1253` | `sample_goal_state`'s guard A (no sampler could be built) | `sample_goal_state_returns_none_when_no_sampler_can_be_built` | discriminating | round-report (in-source comment, round 2/§3a): isolating A alone fails this test, leaves B's test green; comment also notes B is structurally unreachable when A fires (illegal-state-unrepresentable, not just empirical) — verified against current source |
| `planner.rs:1337` | same function, guard B (sampler never converged) | `sample_goal_state_returns_none_when_the_only_candidate_sampler_never_converges` | discriminating | same round-report; isolating B alone fails only this test |

## moveit-planning (4 — override, was p3-acm's)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `pipeline.rs:679` | `generate_plan`'s single `PipelineError::NoPlanners` construction site (nullary variant) | `zero_planners_is_an_error` | single-branch | `rg` enumeration: one construction site (`pipeline.rs:386`); nullary variant carries no payload to lose, so `matches!` cannot be blinder than `==` here. Matches the census's own reconciliation note for this crate |
| `plan_responses.rs:208` | `PlanResponsesContainer` round-trip of a value pushed two lines above in the same test | `plan_responses_container_returns_pushed_outcomes_in_push_order` | not-this-family | direct read: this asserts storage/retrieval fidelity of an `Err` the test itself constructed, not selection among distinct error causes |
| `plan_responses.rs:214` | `shortest_solution`'s `best` accumulator, empty-input case | `shortest_solution_is_none_on_empty_input` | not-this-family | direct read: `best` starts `None` and the loop body never runs on an empty slice — vacuously the only possible outcome, no guard to discriminate |
| `add_time_optimal_parameterization.rs:336` | `adapt`'s single `ResponseAdapterError::Failed` construction site | `adapt_rejects_an_invalid_resample_dt_deferred_from_new` | single-branch | `rg` enumeration: one construction site (line 139) |

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

## UNFIXED

None. No blind assertion was found this round, so there was nothing to
fix.

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
  :199` and `moveit-planners-chomp/trajectory.rs:997` fail clause 3 (the
  test's own `RobotTrajectory::new`/fixture-construction precondition,
  not the subject function's decision); `plan_responses.rs:208` fails
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
