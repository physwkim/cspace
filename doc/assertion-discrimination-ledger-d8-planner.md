# Assertion-discrimination ledger — D8 planner-type unification

Produced by p10-cachedik while building PORTING-PLAN.md D8: merging
`moveit_planners_sbp::registry`'s private `PlanningRequest`/`PlanningResponse`
into `moveit-planning`'s, moving `PlannerManager`/`PlanningContext` into
`moveit_planning::planner`, adding the `moveit-planner-registry` crate, and
wiring `ros/moveit-ros`'s two endpoints to the result.

A second file from the same panel rather than rows appended to
`doc/assertion-discrimination-ledger-cached-ik.md`: that one is scoped, in
its own header, to `moveit-kinematics`' IK-cache persistence and nothing
here touches that crate. It is also not appended to
`doc/assertion-discrimination-ledger-p1-robotmodel.md` (which owns the
pre-existing `moveit-planners-sbp` rows) or
`doc/assertion-discrimination-ledger-p9-ros.md` (which owns
`ros/moveit-ros`), for the reason `reconcile-assertion-ledgers.py`'s
`discover_ledgers` already records: ledgers are globbed and panels own their
own file, so writing into another panel's file across worktrees only
manufactures merge conflicts.

Those two ledgers' *existing* rows were re-anchored this round — D8 shifted
`registry.rs`, `pipeline.rs` and `ros/moveit-ros/src/planning.rs` — and each
of those files carries its own "Re-anchored" note saying by how much and how
the new line was confirmed. This file classifies only the sites D8 *added*.

Verdict legend (same as the other ledgers): **discriminating** (proven to
distinguish ≥2 sibling outcomes, by bite, isolating fixture design, or
structural single-code argument), **single-branch** (exactly one possible
construction site reaches this assertion — nothing to discriminate),
**joint-collapse** (≥2 real sibling guards fire together at this assertion's
fixture and it cannot say which), **not-this-family** (excluded by one of
census §9's three clauses).

Evidence legend: **bite** (a fresh reachability mutation run this round,
reverted after confirming, `git diff --stat` clean), **read** (a structural
argument from the source, stated so it can be checked).

## `moveit-planner-registry` (4)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-planner-registry/src/lib.rs:170` | `resolve_planner`'s single `ok_or_else(Error::unknown_name)` site | `an_unregistered_name_is_rejected_rather_than_defaulted` | single-branch | bite B2 below: swapping the constructed variant to `Error::other` failed this test **and** `registered_planners.rs:81`, and nothing else (3 of 5 still green). One construction site, so there is no sibling cause for the `matches!` to be blind to — what makes the two tests non-redundant is the slice state they run against, not the variant they check |
| `crates/moveit-planner-registry/tests/registered_planners.rs:34` | the `#[distributed_slice(PLANNER_MANAGERS)] static RRT_CONNECT` registration in `moveit-planners-sbp` (`crates/moveit-planners-sbp/src/registry.rs:904-908`, key at `:787`) | `every_expected_registration_exists_regardless_of_slice_order` | discriminating | bite B3 below: renaming the registration key to `"rrt_connect_BITE"` failed this test, `a_registered_name_resolves_to_that_manager` and `registration_names_match_the_managers_they_build`, while `:72`/`:79` and the unit test stayed green. The set membership is checked as a `HashSet`, never by index, so the assertion cannot itself become order-dependent (PORTING-PLAN.md §177) |
| `crates/moveit-planner-registry/tests/registered_planners.rs:74` | the *fixture's own* precondition: `PLANNER_MANAGERS` non-empty | `an_unregistered_name_is_rejected_even_with_registrations_present` | not-this-family | census §9: it guards the fixture, not a failure branch of production code — but it is not decorative, and bite B4 is what shows that. Deleting the `use moveit_planners_sbp as _;` link anchor emptied the slice; this line failed, and in the same run `registration_names_match_the_managers_they_build` **passed vacuously** on the now-empty loop. That vacuous pass is exactly the outcome this line refuses for `:79` |
| `crates/moveit-planner-registry/tests/registered_planners.rs:81` | `resolve_planner`'s name lookup, against a *populated* slice | `an_unregistered_name_is_rejected_even_with_registrations_present` | discriminating | bite B1 below: adding `.or_else(\|\| PLANNER_MANAGERS.iter().next())` — the fallback-to-first-entry bug this test exists for — failed this test **alone**; the other four, including `crates/moveit-planner-registry/src/lib.rs:170`'s empty-slice copy of the same assertion, stayed green. That is the measured difference between the two: an empty slice has no first entry to fall back to, so `crates/moveit-planner-registry/src/lib.rs:170` cannot see this bug at all |

## `moveit-planners-sbp` (1)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-planners-sbp/src/registry.rs:1287` | `RrtConnectContext::solve_inner`'s single `PlanError::NoGoalConstraints` site (`crates/moveit-planners-sbp/src/registry.rs:799`), the empty-goal-set branch of `ModelBasedPlanningContext::setGoalConstraints` (`model_based_planning_context.cpp:690-694`) | `an_all_empty_goal_constraint_list_is_rejected_before_sampling` | discriminating | bite B5 below: changing that `return Err` to its nearest sibling `PlanError::NoGoalSample` — the other way a goal can produce nothing — failed this test **alone**, 111 of 112 green. The test's own doc comment names that confusion as the thing it exists to prevent, and the bite is what makes that a checked claim |

## `ros/moveit-ros` (1)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `ros/moveit-ros/src/move_group.rs:385` | `resolve_planning_pipeline`'s non-empty-`pipeline_id` branch (`move_group_capability.cpp:230-245`: look the name up, return null on a miss) | `a_named_pipeline_id_is_looked_up_verbatim` | discriminating | bite B7 below: making the lookup fall back to `DEFAULT_PIPELINE_ID` on a miss — upstream's null return replaced by a silent substitution — failed this line and `an_unregistered_pipeline_id_fails_before_any_planning_runs`, while `an_empty_pipeline_id_resolves_to_the_named_default` and both `plan_only` tests stayed green. Two tests, not one: the fallback is a single guard both observe, from opposite sides |

## Bites

Every mutation below was applied, run, reverted, and the revert confirmed —
by `git diff --stat` for tracked files and by re-reading the mutated line for
the three files this round creates (`crates/moveit-planner-registry/**`,
`ros/moveit-ros/src/move_group.rs`), which are untracked at the time of the
bite and so cannot be restored with `git checkout --`. That asymmetry cost
one contaminated run in this round: B1's mutation survived a `git checkout --`
that silently did nothing on an untracked file, and B2 was first measured
with both mutations live. B2's row above is the re-run, with B1 removed.

* **B1** `resolve_planner`: insert `.or_else(|| PLANNER_MANAGERS.iter().next())`
  before the `ok_or_else`. → `an_unregistered_name_is_rejected_even_with_registrations_present`
  FAIL; 4 PASS.
* **B2** `resolve_planner`: `Error::unknown_name("planner manager", name)` →
  `Error::other(format!(...))`. → `an_unregistered_name_is_rejected_rather_than_defaulted`
  and `an_unregistered_name_is_rejected_even_with_registrations_present` FAIL;
  3 PASS.
* **B3** `moveit-planners-sbp`'s registration: `name: "rrt_connect"` →
  `"rrt_connect_BITE"`. → `every_expected_registration_exists_regardless_of_slice_order`,
  `a_registered_name_resolves_to_that_manager`,
  `registration_names_match_the_managers_they_build` FAIL; 2 PASS.
* **B4** `tests/registered_planners.rs`: delete `use moveit_planners_sbp as _;`.
  → `every_expected_registration_exists_regardless_of_slice_order`,
  `a_registered_name_resolves_to_that_manager`,
  `an_unregistered_name_is_rejected_even_with_registrations_present` FAIL;
  `registration_names_match_the_managers_they_build` PASSES on an empty loop.
* **B5** `crates/moveit-planners-sbp/src/registry.rs:799`: `PlanError::NoGoalConstraints` → `PlanError::NoGoalSample`.
  → `an_all_empty_goal_constraint_list_is_rejected_before_sampling` FAIL;
  111 PASS.
* **B6** `joint_model_group_space.rs:132`: `SbpError::UnknownGroup { .. }` →
  `SbpError::NoSubspaces`. → `registry::tests::unknown_group_is_rejected_before_any_search_runs`
  and `joint_model_group_space::tests::unknown_group_is_rejected` FAIL; 101 PASS.
  Not a new site — this re-runs the bite
  `doc/assertion-discrimination-ledger-p1-robotmodel.md`'s
  `crates/moveit-planners-sbp/src/registry.rs:1251` row cites, because D8
  rewrote that assertion: `moveit_planning::PlanError` is
  `Box<dyn Error + Send + Sync>`, so the concrete variant is now reached by
  `downcast_ref` rather than matched directly, and a bite recorded against
  the old shape would not have covered the new one.
* **B7** `move_group.rs`: `resolve_planner(name).ok()` →
  `resolve_planner(name).or_else(|_| resolve_planner(DEFAULT_PIPELINE_ID)).ok()`.
  → `a_named_pipeline_id_is_looked_up_verbatim`,
  `an_unregistered_pipeline_id_fails_before_any_planning_runs` FAIL; 3 PASS.
* **B8** `plan_only`: return `PlanOnlyError::Planning(PipelineError::NoPlanners)`
  in place of `PlanOnlyError::UnknownPipeline`. →
  `an_unregistered_pipeline_id_fails_before_any_planning_runs` FAIL; 4 PASS.
  This is what makes the two-variant split checked rather than merely
  written: upstream reports both as `MoveItErrorCodes::FAILURE`, so only the
  message distinguishes them for a caller.
* **B9** `crates/moveit-planners-sbp/src/registry.rs:898`: `planner_id: "rrt_connect".to_string()` →
  `"BITE"`. → `move_group::tests::the_plan_only_arm_reaches_rrt_connect_and_gets_a_trajectory`
  FAIL alone (4 PASS) and, in the root workspace,
  `registry::tests::end_to_end_solve_on_panda_arm_reaches_the_requested_goal`
  FAIL alone (161 PASS). The round's success criterion is that the plan-only
  arm reaches *this* planner; a trajectory alone would look identical if some
  other registration had answered, and `planner_id` is the only field that
  says which one did.
* **B10** `pipeline.rs`'s `run_planner`: map a `get_planning_context` failure
  to `PipelineError::NoPlanners` instead of `PipelineError::Planner`. →
  `context_construction_failure_is_reported_as_a_planner_failure` FAIL; 121
  PASS. `moveit-planners-sbp` reaches this branch for real (an unknown group
  is rejected in `get_planning_context`, not in `solve`), so it is not a
  double-only path.

The `move_group.rs` bites (B7, B8, B9's second half) run under
`sg docker -c 'docker run ... moveit-rs/ros-dev:latest cargo test move_group'`:
`ros/moveit-ros` is its own workspace (D5) and does not compile outside that
image.
