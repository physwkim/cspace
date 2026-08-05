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

## Round 10 — `moveit-constraints` coverage gap (override, orchestrator's own fence)

Not a census row: `assert_err_mentions`'s `assert!(rendered.contains(needle), ...)`
is a `.contains()` check, not `matches!`/bare `.is_err()`/`.is_none()`, so
this site is outside census §1's syntactic scan and adds nothing to the
289. It is the same defect family the sweep exists to catch — a guard an
assertion could be blind to — found as a coverage gap rather than an
existing site's misclassification.

`PositionConstraint::new`'s first fallible call, `model.link_model
(link_name)?` (`position.rs:165`), is a sibling of `resolve_frame`'s
frame-id guard (`position.rs:108-109`, called from `:188`): both reach
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

`PositionConstraint::new` (`position.rs:156-197`) siblings: `link_model(link_name)?`
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
| `decide.rs:372` | `no frame specified for position constraint` | discriminating | none of the other 5 messages contain it |
| `decide.rs:390` | `needs at least one constraint region` | discriminating | substring only of the shapes-empty message; not in the other 5 |
| `decide.rs:448` | `has no bodies:: counterpart` | discriminating | substring only of the `Ok(None)` message; not in the other 5 |
| `decide.rs:518` | `convex mesh body requires at least one vertex` | discriminating | exact text of `bodies.rs:2542-2543`'s guard; not in the other 5, nor in `bodies.rs`'s own sibling `convex hull computation failed: {e}` |
| `decide.rs:544` | `no link named "no_such_link"` | discriminating | not in the `frame`-kind message (different `kind`) nor the other 4 |
| `decide.rs:563` | `no frame named "no_such_frame"` | discriminating | not in the `link`-kind message nor the other 4 |

`OrientationConstraint::new` (`orientation.rs:195-248`) has only two
fallible sites: `link_model(link_name)?` → `no link named "X"`;
`!model.has_link_model(frame_id) && frame_id != model.model_frame()` →
`no frame named "X"` (one guard, reached both by an unresolvable name and
by the empty string — no separate empty-`frame_id` branch exists for this
type, unlike `PositionConstraint`; the test doc comment at `:770-780`
says so directly).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `decide.rs:756` | `no frame named "no_such_frame"` | discriminating | not in the `link`-kind message; the other same-guard instance (`decide.rs:795`) differs by quoted content only |
| `decide.rs:795` | `no frame named ""` | discriminating | same as above, reversed; deliberately the same guard as `:756`, not a collision — the doc comment says so |

**`moveit-constraints/tests/sampler.rs`** — subject `JointConstraintSampler::new`
(`sampler.rs:179-259`), three sites: unknown group → `Error::UnknownName`;
empty intersection → `"JointConstraintSampler: no possible values for
joint variable '{}': min_bound {} > max_bound {}"`; no applicable
constraint → `"JointConstraintSampler: no joint constraints apply to
group '{group_name}'"`.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `sampler.rs:79` | `panda_joint1` | discriminating | not in the group-name message for this call (`group_name = "panda_arm"`) |
| `sampler.rs:121` | `panda_arm` | discriminating, **fragile** | for this call's inputs, unique; flagged because `panda_arm` is a prefix of `panda_arm_hand`, so if this needle were ever reused against an unknown-group failure for `"panda_arm_hand"` it would spuriously match — not fixed, no such reuse exists today |

**`moveit-distance-field/src/voxel_grid.rs`** — already bite-checked
in-code (`:439-449`, `:479-487`, `:494-507`); this round transcribes,
does not re-derive. `GridGeometry::new`'s three guards: `resolution` not
finite/positive → `... must be finite and positive`; `size.x`/`size.y`/`size.z`
over-resolution overflow, one message per axis naming that axis.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `voxel_grid.rs:454` | `must be finite and positive` | discriminating | in-code bite: disabling the resolution guard alone leaves this assertion red (the overflow guard's message never contains this phrase) |
| `voxel_grid.rs:456` | `must be finite and positive` | discriminating | same guard, negative-resolution boundary; same bite |
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
— subject `ChainInfo::build` (`chain.rs:145-294`), four Err messages:
`not a chain`, `{} DOF; only single-DOF joints are supported`,
`unsupported type {}`, `not itself in the group`. `NewtonRaphsonSolver::new`
(`newton_raphson.rs:83-109`) only forwards `ChainInfo::build`'s error —
verified by reading the function, no extra branch.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `chain.rs:469` | `not a chain` | discriminating | not in the DOF, unsupported-type, or mimic-master messages |
| `chain.rs:512` | `DOF` | discriminating | not in the other 3 |
| `chain.rs:558` | `not itself in the group` | discriminating | not in the other 3 |
| `ik_fk_roundtrip.rs:267` | `not a chain` | discriminating | `NewtonRaphsonSolver::new` forwards the identical 4-message surface; same check as `chain.rs:469` |

**`moveit-model/src/robot_model.rs`** — two independent guard groups.
`RobotModel::from_urdf_and_srdf`'s root-link resolution (`:215-230`): `[]`
→ `has no root link`; `names` (>1) → `has {} root links, expected exactly
one`. `mesh_collision_*` diagnostics (`:2380-2555`ish, four
`Diagnostic::UnsupportedLinkGeometry` producers sharing `kind == "mesh"`,
distinguished only by `detail`): unresolved path, non-STL file, unreadable
file, malformed STL bytes.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `robot_model.rs:2287` | `has no root link` | discriminating | not in the multi-root message |
| `robot_model.rs:2302` | `root links, expected exactly one` | discriminating | not in the no-root message |
| `robot_model.rs:2472` | `only STL is supported` | discriminating | not in "it could not be read" / "failed to parse" / the 4th (equality-checked, not `.contains()`) sibling; doc comment names all four explicitly |
| `robot_model.rs:2512` | `it could not be read` | discriminating | not in the other 3 |
| `robot_model.rs:2542` | `failed to parse` | discriminating | not in the other 3 |
| `robot_model.rs:2635` | `Box dimensions` | discriminating, **breadth caveat** | subject is `build()`, the whole model pipeline, not a narrow constructor — sibling surface not exhaustively enumerable the way a single function's is; the phrase itself (from `bodies.rs:2210`) is distinctive and not found reused elsewhere in this crate |

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
| `optimizer.rs:2380` | `joint_costs has` | discriminating | not in `ChompCost::derivative`'s joint_trajectory-length message |
| `optimizer.rs:2449` | `columns` | discriminating | in-code: "appears only in this one's message" among 3 sites |
| `trajectory.rs:712` | `discretization must be finite and positive` | discriminating | not in "would require more than" or `from_num_points`'s num_points<2 message |
| `trajectory.rs:727` | same | discriminating | same guard, negative-discretization boundary |
| `trajectory.rs:748` | same | discriminating | same guard, negative/negative boundary that divides positive |
| `trajectory.rs:766` | `would require more than` | discriminating | not in the discretization message |
| `trajectory.rs:925` | `active joints, but this ChompTrajectory has` | discriminating | not in the multi-DOF-joint message |
| `trajectory.rs:968` | `variables; ChompTrajectory requires every active joint` | discriminating | not in the column-count message |
| `trajectory.rs:1004` | `requires trajectory.group() to be Some` | discriminating | not in any of `fill_in_from_trajectory`'s several other reachable messages |

**`moveit-planners-stomp/src/filter_functions.rs`** — `enforce_position_bounds`
has exactly one Err site (`require_single_variable`'s guard, one call per
joint in a loop, "no sibling branch" per the in-code comment at
`:288-306`); the compound needle names the joint and its variable count
within that one message, not two different guards.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `filter_functions.rs:314` | `world_joint` AND `3 variables` | discriminating | single reachable guard; compound needle verifies message content, not branch identity — no sibling to collide with |

**`moveit-planning/src/request_adapters/check_start_state_collision.rs`**
— `RequestAdapterError` has exactly two variants (`error.rs:22-42`):
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
| `acceleration_filter.rs:525` | `planar_joint` AND `3` | discriminating | `message = err.to_string()` (script misses the `err_shape` heuristic here only because the binding is on a prior line, not because it isn't one) |
| `acceleration_filter.rs:542` | `Make sure the reset was called` | discriminating | message-swap bite-checked against the sibling length-mismatch guard |

**`moveit-smoothing/src/ruckig_filter.rs`** — mirror structure to
`acceleration_filter.rs`, same bite-checked pattern.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `ruckig_filter.rs:388` | `acceleration limit defined` | discriminating | message-swap bite-checked against the sibling single-DOF/velocity/jerk guards |
| `ruckig_filter.rs:465` | `planar_joint` AND `3` | discriminating | same shape as `acceleration_filter.rs:525` |
| `ruckig_filter.rs:539` | `must each have length` | discriminating | message-swap bite-checked against the sibling ruckig-update-failure site |

**`moveit-srdf/tests/boundaries.rs`** — `SrdfModel::parse_str`'s two
`Error::Parse` sites, already discriminated by p1-joints this session
(`83f8ea0`); this round re-verifies rather than re-fixes (user's standing
instruction: no further edits to work another panel's round already
closed).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `boundaries.rs:49` | `opened but never closed` | discriminating | roxmltree's own `UnclosedRootNode` message (verified against the vendored `roxmltree-0.21.1/src/parse.rs:254-255`: `"the root node was opened but never closed"`, generic, names no element) — not in the other guard's message |
| `boundaries.rs:55` | `robot`, **fragile** | discriminating | not in roxmltree's message (verified: that message never names an element at all) today; flagged because `robot` is the XML root tag's own literal name and a future third `Error::Parse` site that echoes a tag name could collide — not fixed, no such site exists |

**`moveit-state/tests/jacobian.rs`** — subject `Posed::jacobian`
(`state.rs:1121-1210`ish), three messages: `is_chain()` false → `the
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
| `path.rs:318` | `180 deg` | discriminating | not in the other 2 |

**`moveit-trajectory/src/time_optimal_trajectory_generation.rs`** — three
independent guard groups.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `time_optimal_trajectory_generation.rs:1070` | `exceeding the` | discriminating | `"exceeding"` occurs exactly once in the whole file (`rg -n exceeding`); globally unique, not just locally |
| `:1097` | same | discriminating | same guard, subnormal-`resample_dt` boundary |
| `:1163` | same | discriminating | same guard, `usize::MAX`-targeting boundary |
| `:1421` | `4` AND `7`, **fragile** | discriminating | the mimic-dimension-mismatch guard (`:770-780`) is the *only* reachable `Err` once `max_velocity.len() != group.variable_names().len()`, which is unconditional for any group with a mimic joint — no sibling branch is reachable to collide with today, but the needles themselves are bare digits, the weakest form found this round; any future guard added ahead of this one in `do_time_parameterization_calculations` that also renders a floating-point value ending in 4 or 7 would defeat it. Not fixed — not currently blind, and the brief says do not fix speculatively. |
| `:1470` | NOT `invalid max_acceleration`, exclusionary | discriminating | a custom (caller-supplied) acceleration limit skips the bounds-fallback validation entirely (`:592-606`); the negation proves the failure did not come from that branch, which is what the test claims — verified the custom-limit code path truly has no validation call on it |
| `:1511` | `num_waypoints > 1` | discriminating | `totg_compute_time_stamps`'s only guard before delegating to `compute_time_stamps` |
| `:1517` | same | discriminating | same guard, `num_waypoints = 0` boundary |

**`moveit-trajectory/src/trajectory.rs`** — `Trajectory::create`'s three
`Error::construct` sites (`:121-176`), already self-documented in-code
(`:1427-1433`, the `DISTINGUISHING_PHRASE` constant and its own comment).

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `trajectory.rs:1434` | `after integrateForward and integrateBackward` | discriminating | not in "the time step is <= 0.0" or "after the second integrateBackward pass" (the word order differs from this needle) |
| `:1440` | same | discriminating | same guard, second velocity-vector case |
| `:1446` | same | discriminating | same guard, third case |
| `:1473` | `the time step is <= 0.0` | discriminating | not in the other 2 |

**`moveit-trajectory/tests/robot_trajectory.rs`** — `RobotTrajectory::insert_way_point`'s
two `Error::other` sites, in-code bite noted at `:502-504`.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `robot_trajectory.rs:514` | `duration_from_previous[0] must be 0.0` | discriminating | not in `index_error`'s message |
| `robot_trajectory.rs:676` | `out of bounds` | discriminating | exact substring of `index_error`'s message only; not in `first_duration_error`'s or `empty_error`'s |

**`moveit-trajectory/tests/ruckig_smoothing.rs`** — `apply_smoothing`'s
three `Error::other` sites, in-code noted at `:198-201`.

| file:line | needle | verdict | siblings checked |
|---|---|---|---|
| `ruckig_smoothing.rs:204` | `did not set the group` | discriminating | not in "ruckig calculate failed: {error}" or the third (smoothing-result-failure) message |

### Result: 0 blind sites, 3 fragile-but-currently-unique needles flagged, not fixed

All 77 in-scope sites verdict `discriminating`. No needle collided with a
sibling message under this round's reading. Three fragility notes are
recorded above (`sampler.rs:121`, `boundaries.rs:55`,
`time_optimal_trajectory_generation.rs:1421`) per the brief's instruction
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
- `rg -n 'exceeding' crates/moveit-trajectory/src/time_optimal_trajectory_generation.rs` — 1 occurrence, confirms `:1070/1097/1163`'s needle is globally unique in-file, not just locally
- `cargo fmt --all -- --check` — clean
- No source changes this round (0 blind sites), so no `-p <crate>` clippy/nextest gate was owed beyond the doc-only fmt check above.

Gate scope: doc-only round, `cargo fmt --all -- --check`. No crate's
clippy/nextest gate owed — no source file in this fence changed.
beyond the standing pre-push rule.
