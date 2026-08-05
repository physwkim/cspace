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
  `utils.rs:499`): `check_scenario` asserts `.is_none()`/`.is_some()`
  on `actual.voxel`, `rows_to_string` asserts `!rows.is_empty()`.
  Neither renders or checks an error *message* — they are
  Option-presence and collection-emptiness checks, a different
  assertion family from this round's ("`assert_err_mentions`-style and
  bare `.to_string().contains(...)`" — message-substring
  discrimination only). Their 4 call sites are not this fence's family.

  65 + 12 = 77 with no double count between those two buckets (`kind`
  is either exactly `contains_msg` or starts with `via:`, never both).
  The reported 81 is 77 plus the 4 `helper_body` lines (`decide.rs:83`,
  `oracle_parity.rs:264,279`, `utils.rs:499`) added back on top —
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
misfiled `body.touch_links().contains(...)` at `ros/.../attached.rs:532`)
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
| `decide.rs:1161,1166` (`assert_eq!(..., None)` on `max_view_angle()`/`max_range_angle()`) | yes | discriminating | isolating mutation run this round: `normalize_angle_criterion`'s `.filter(\|v\| *v > EPS)` gate removed → `negative_target_radius_activates_...` and `zero_valued_criteria_normalize_to_unconstrained` both fail (2/100), no other test moves; `target_radius`'s own gate (`normalize_target_radius`, a different fn) is untouched and its sibling assertion in the same test stays green |
| `decide.rs:1257` (`set.is_empty()`) | **no** | not-this-family | `KinematicConstraintSet::new()` is `Self::default()` (`set.rs:54-56`) — trivially empty by construction, clause 2 fails (nothing was decided) |

**`crates/moveit-model/src/joint/model.rs` (3)**

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `model.rs:946` (`local_variable_names().is_empty()`, single-variable joint) | yes | discriminating | cross-test sibling `multi_variable_joints_prefix_local_names_with_the_joint_name` shows 7 non-empty names for a floating joint |
| `model.rs:975` (`variable_names().is_empty()`, fixed joint) | yes | discriminating | cross-test siblings show 1 name (revolute, `:944`) and 7 names (floating, `:948-955`) |
| `model.rs:976` (`variable_bounds().is_empty()`, fixed joint) | yes | discriminating | cross-test siblings index `variable_bounds()[0]` directly at `:1008,1031,1074,1087,1105` — non-empty for every other joint kind tested |

**`crates/moveit-model/src/mesh_search_paths.rs` (5)**

`resolve()` has four independent `None`-producing guards (`?` on
`strip_prefix("package://")`, `?` on `split_once('/')`, `?` on
`self.packages.get(package)`, and `candidate.is_file()` false).

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `mesh_search_paths.rs:108` (`paths.is_empty()`, on `none()`) | **no** | not-this-family | `is_empty()` on `packages.is_empty()`, `none()` on `Self::default()` — both trivial, no decision links them |
| `mesh_search_paths.rs:109` (`none_resolves_nothing`) | yes | discriminating | empty map — isolates the `self.packages.get(package)?` guard; distinguishable from `:139/140`'s guard (see below) |
| `mesh_search_paths.rs:130` (`unknown_package_does_not_resolve`) | yes | discriminating | map has one real entry, queried key absent — same guard as `:109` but proves it is the lookup miss, not "map empty," that fires |
| `mesh_search_paths.rs:139,140` (`non_package_uri_does_not_resolve`) | yes | discriminating | neither input has a `"package://"` prefix — isolates `strip_prefix(...)?`, distinct guard from `:109/130` |

**Not fixed, flagged rather than fabricated**: the `split_once('/')?`
guard (malformed `package://name` with no `/`) and the
`candidate.is_file()` guard (known package, well-formed path, file
missing) have **no test at all** — not a blind existing assertion, an
absent one. Writing new tests is outside "fix the blind ones"; this is
a coverage gap, not a fix owed this round. Reporting it rather than
silently dropping it.

**`crates/moveit-model/src/robot_model.rs` (24)**

| file:line | member? | verdict | evidence |
|---|---|---|---|
| `robot_model.rs:1972` (`mimic_chain_collapses_transitively`, diagnostics-empty) | yes | discriminating | cross-test siblings: `MimicCycle` positive at `:2051`, `MimicUnknownJoint` at `:2073`, `MimicDofMismatch` at `:2105` |
| `robot_model.rs:2260` (`only_j1`'s `subgroup_names().is_empty()`) | yes | discriminating | real per-candidate comparison against `only_j2`/`all` (not vacuous — candidates exist); same test's `all.subgroup_names()` (`:2259`) is the non-empty sibling |
| `robot_model.rs:2287,2302` | yes | discriminating | round 11, unchanged |
| `robot_model.rs:2369` (box collision, diagnostics-empty) | yes | discriminating | cross-test siblings: capsule/mesh/negative-dimension diagnostic tests below it in the same file |
| `robot_model.rs:2392` (`base.shapes().is_empty()`, link with zero `<collision>` elements) | **no** | not-this-family | vacuous — `link_with_geometry_urdf("")` has no collision element at all, nothing for the shape-construction loop to iterate; same shape as the census's `shortest_solution_is_none_on_empty_input` example |
| `robot_model.rs:2411,2518` (mesh skipped, `shapes().is_empty()`) | yes | discriminating | real skip guard (path resolution/unreadable file), cross-test sibling `:2561-2564` shows `shapes.len()==1`, `Shape::Mesh(_)` when resolution succeeds |
| `robot_model.rs:2472,2512,2542` | yes | discriminating | round 11, unchanged |
| `robot_model.rs:2561` (valid stl, diagnostics-empty) | yes | discriminating | positive-case anchor for `:2411/2518/2472/2512/2542`; cross-test siblings show diagnostics populated for every failure shape |
| `robot_model.rs:2635` | yes | discriminating, breadth caveat | round 11, unchanged |
| `robot_model.rs:2676` (`visual_mesh_filename()`, `None`) | yes | discriminating | cross-test siblings `:2650` (`Some("visual.dae")`), `:2666` (`Some("collision.stl")`) |
| `robot_model.rs:2764` (`"other"` group, `attached_end_effector_names().is_empty()`) | yes | discriminating | same test: `"arm"`/`"full_arm"` (`:2755,2762`) are the non-empty siblings; real ancestry-walk decision, not vacuous |
| `robot_model.rs:2885` (`end_effector_parent()`, `None`) | yes | discriminating | cross-test siblings `:2743,2813,2832,2851,2869` all `Some(EndEffectorParent{..})` |
| `robot_model.rs:2886` (`attached_end_effector_names().is_empty()`, no end effector declared at all) | yes | discriminating | cross-test sibling `:2755,2762` (`["grasper"]`) |
| `robot_model.rs:2986` (unknown-group `group_state`, ignored) | yes | discriminating | cross-test sibling `:2936` (`["home"]`); real group-name-lookup guard, distinct decision from `:3021`'s |
| `robot_model.rs:3021` (every joint value unusable, no state stored) | yes | discriminating | real per-value dimension-mismatch guard (not vacuous — one `<group_state>` element present); cross-test sibling `:2936` |
| `robot_model.rs:3031` (`variable_default_positions_returns_none_for_unknown_state_name`, empty srdf) | **no** | not-this-family | vacuous — `group_state_test_srdf("")` has zero `<group_state>` elements, nothing for the collection loop to iterate |
| `robot_model.rs:3376-3379` (fixed-joints-only group: `active_joint_indices`/`joint_roots`/`updated_link_names`/`updated_link_with_geometry_names`, all empty) | yes | single-branch | doc comment (`:3341-3350`) gives the causal chain explicitly; each is a real per-joint/per-root decision (not vacuous — 2 fixed joints present), but only this one fixture shape is exercised in this fence, no direct `.is_empty()`-shaped sibling asserting non-empty for the same getters |

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
| `check_start_state_bounds.rs:302` (`an_out_of_bounds_velocity_is_rejected`) | yes | discriminating | **isolating mutation, run this round**: `if false && state.has_velocities() && !joint.satisfies_velocity_bounds(...)` → this test fails (`Ok(())` vs expected `Err`), `:328`'s test stays green |
| `check_start_state_bounds.rs:328` (`a_joint_placed_outside_its_limits_is_rejected_regardless_of_fix_start_state`) | yes | discriminating | **isolating mutation, run this round**: `if false && !joint.satisfies_position_bounds(...)` → this test fails, `:302`'s test stays green |
| `check_start_state_bounds.rs:377` (`(-PI..=PI).contains(&wrapped)`) | **no** | not-this-family | range check on a directly computed numeric result (the wrap itself), not an absence/failure signal — same exclusion as `sampler.rs`'s bounds checks in round 11 |
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

### Result: 0 blind sites; 8 of 52 ruled `not-this-family`; one coverage gap flagged, not fixed

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
| `mesh_search_paths.rs:130` (`unknown_package_does_not_resolve`) | yes | discriminating | **corrected: was blind, now discriminating** | isolating mutation on `:88` (see table above) — round 13's verdict rested on reading, not mutation, and was wrong until the fixture was rebuilt |
| `mesh_search_paths.rs:139,140` (`non_package_uri_does_not_resolve`) | yes | discriminating | **corrected: was blind, now discriminating** | isolating mutation on `:86` (see table above) — same correction |
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
