# Assertion-discrimination ledger — p10-jointlimits (pilz `joint_limits_aggregator`, `joint_limits_validator`)

The two modules this panel adds carry 39 new tests. The coarse-assertion
scanner emits exactly **one** site for them —
`crates/moveit-planners-pilz/src/joint_limits_aggregator.rs:405` (`an_empty_joint_list_aggregates_to_an_empty_container`) — because
every other assertion in both modules is an `assert_eq!` against a named
value or an `assert!` on a named `bool` field. §1 accounts for that one
site.

§2 and §3 go past the scanner corpus: they record an isolating mutation for
every guard the two modules contain, so that "this test covers that guard"
is a measured claim rather than a reading of the test's name. Of the 39
tests, **38 have a mutation that fails that test and no other**; the one
that does not is named in §4 with the reason no such mutation can exist.

## Method

Every row's evidence is a single textual mutation applied to this worktree,
run with `cargo nextest run -p moveit-planners-pilz --no-fail-fast`
(249 tests, 249 passing at baseline), then reverted by rewriting the
pre-mutation text and confirmed byte-identical. No row is justified by
reading the code, and none by a run from an earlier round.

Two mutation ids are unused: `A25`, allocated to an attempt at an exclusive
mutation for `a_continuous_joint_accepts_any_position_override` that `A31`
later supplied, and nothing else. Ids are not renumbered, so that a reader
re-running a mutation gets the same id it carries here.

Mutations that fail to compile are not evidence. Two first drafts did —
`if false {` in `check_velocity_bounds` and dropping `overrides` in
`aggregate_limits` both left an unused binding — and were replaced with the
compiling forms recorded below (`A6`, `A17`) before any verdict was taken.

## 1. The one scanner site

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs:405` | `assert!(aggregated.is_empty())` — with no joint models, `aggregate_limits` returns a container holding nothing | `an_empty_joint_list_aggregates_to_an_empty_container` | discriminating | Bite `A16`: append `if container.is_empty() { let _ = container.add_limit("phantom", JointLimit::default()); }` before `Ok(container)`. That one test FAILS and the other 248 stay green. The mutation is synthetic by necessity — the empty path has no operand of its own to move, because the `for` loop simply does not run — but it is the exact defect the assertion exists to catch: an aggregator that manufactures an entry no joint asked for. The companion claim, that the container is keyed by the joint's own name rather than by anything else, is `A21` under §2 and fails a different single test. |

## 2. `joint_limits_aggregator.rs` — 22 tests, 22 exclusive mutations

Guard-by-guard. "fails" lists every test that fails under the mutation;
a row with one name is an isolating mutation for that test.

| id | mutation | fails |
|---|---|---|
| `A1` | `check_bounds_check_is_supported`: `variable_count > 1` → `> 100` (guard never fires) | `a_position_override_on_a_multi_variable_joint_is_rejected`, `a_velocity_override_on_a_multi_variable_joint_is_rejected` |
| `A2` | same guard: `> 1` → `> 0` (fires for single-variable joints too) | 9: `a_continuous_joint_accepts_any_position_override`, `a_max_position_above_the_models_is_rejected`, `a_max_velocity_above_the_models_is_rejected`, `a_min_position_below_the_models_is_rejected`, `a_negative_max_velocity_below_the_models_min_is_rejected`, `a_position_override_equal_to_the_models_bounds_is_accepted`, `a_stricter_position_override_replaces_the_models_window`, `a_stricter_velocity_override_replaces_the_models_bound`, `every_joint_gets_exactly_one_limit` |
| `A3` | the guard's call site condition → `true` (guard scoped to the joint, not to the checks) | `a_multi_variable_joint_with_no_override_is_pinned_to_zero`, `an_acceleration_only_override_on_a_multi_variable_joint_is_allowed` |
| `A4` | `check_position_bounds`: the `min_position` test → `if false` | `a_min_position_below_the_models_is_rejected` |
| `A5` | `check_position_bounds`: the `max_position` test → `if false` | `a_max_position_above_the_models_is_rejected` |
| `A6` | `check_velocity_bounds`: `if false && !joint_model.satisfies_velocity_bounds(...)` | `a_max_velocity_above_the_models_is_rejected`, `a_negative_max_velocity_below_the_models_min_is_rejected` |
| `A7` | rule 4 drops its `&& !joint_limit.has_deceleration_limits` conjunct | `an_explicit_deceleration_override_survives_rule_four` |
| `A8` | rule 4 drops its `joint_limit.has_acceleration_limits &&` conjunct | `a_joint_with_no_override_takes_the_models_bounds_and_nothing_else`, `a_joint_with_no_variables_contributes_nothing` |
| `A9` | `if container.has_limit(name)` → `if false` (the ordering that makes `add_limit`'s `false` attributable) | `a_repeated_joint_is_rejected_rather_than_dropped` |
| `A10` | the `add_limit` block → `let _ = container.add_limit(name, joint_limit);`, i.e. upstream's discarded `bool` | `a_zero_acceleration_override_is_reported_not_dropped` |
| `A11` | `update_position_limit_from_joint_model`'s `[]` arm sets `has_position_limits = true` instead of changing nothing | `a_joint_with_no_variables_contributes_nothing` |
| `A12` | its `[bounds]` arm: `has_position_limits = bounds.position_bounded` → `= true` | `an_unbounded_model_position_still_contributes_its_numbers` |
| `A13` | its `_` arm → `{}` | `a_multi_variable_joint_with_no_override_is_pinned_to_zero`, `an_acceleration_only_override_on_a_multi_variable_joint_is_allowed` |
| `A14` | `update_velocity_limit_from_joint_model`'s `[bounds]` arm: `has_velocity_limits = bounds.velocity_bounded` → `= true` | `an_unbounded_model_position_still_contributes_its_numbers` |
| `A15` | its `_` arm → `{}` | `a_multi_variable_joint_with_no_override_is_pinned_to_zero`, `an_acceleration_only_override_on_a_multi_variable_joint_is_allowed` |
| `A16` | a phantom entry is added when the joint list was empty (§1) | `an_empty_joint_list_aggregates_to_an_empty_container` |
| `A17` | `overrides.limit(name).unwrap_or_default()` → `{ let _ = overrides; JointLimit::default() }` (the override set ignored entirely) | 14: every test that supplies an override — `a_continuous_joint_accepts_any_position_override`, `a_deceleration_override_alone_leaves_acceleration_undefined`, `a_max_position_above_the_models_is_rejected`, `a_max_velocity_above_the_models_is_rejected`, `a_min_position_below_the_models_is_rejected`, `a_negative_max_velocity_below_the_models_min_is_rejected`, `a_position_override_on_a_multi_variable_joint_is_rejected`, `a_stricter_position_override_replaces_the_models_window`, `a_stricter_velocity_override_replaces_the_models_bound`, `a_velocity_override_on_a_multi_variable_joint_is_rejected`, `a_zero_acceleration_override_is_reported_not_dropped`, `an_acceleration_only_override_on_a_multi_variable_joint_is_allowed`, `an_acceleration_override_alone_derives_the_deceleration_limit`, `an_explicit_deceleration_override_survives_rule_four` |
| `A18` | the position helper stops copying `min_position`/`max_position` | `a_joint_with_no_override_takes_the_models_bounds_and_nothing_else`, `an_unbounded_model_position_still_contributes_its_numbers` |
| `A19` | the velocity helper stops copying `max_velocity` | `a_joint_with_no_override_takes_the_models_bounds_and_nothing_else`, `an_unbounded_model_position_still_contributes_its_numbers` |
| `A20` | `if joint_limit.has_position_limits` → `if false` (a position override is never checked and always overwritten by the model) | `a_continuous_joint_accepts_any_position_override`, `a_max_position_above_the_models_is_rejected`, `a_min_position_below_the_models_is_rejected`, `a_stricter_position_override_replaces_the_models_window` |
| `A21` | `container.add_limit(name, …)` → `add_limit("j1", …)` (keyed by a constant) | `every_joint_gets_exactly_one_limit` |
| `A22` | the position helper's `[bounds]` arm: `= bounds.position_bounded` → `= false` | `a_joint_with_no_override_takes_the_models_bounds_and_nothing_else` |
| `A23` | the position helper's `_` arm: `min_position = 0.0` → `= 1.0` | `a_multi_variable_joint_with_no_override_is_pinned_to_zero` |
| `A24` | `check_position_bounds`' `min_position` test with margin `-1e-9` (window made exclusive) | `a_max_position_above_the_models_is_rejected`, `a_position_override_equal_to_the_models_bounds_is_accepted` |
| `A26` | `joint_limit.max_velocity = 0.0` inserted after a *successful* `check_velocity_bounds` | `a_stricter_velocity_override_replaces_the_models_bound` |
| `A27` | rule 4: `= -max_acceleration` → `= -max_acceleration - 1.0` | `a_zero_acceleration_override_is_reported_not_dropped`, `an_acceleration_only_override_on_a_multi_variable_joint_is_allowed`, `an_acceleration_override_alone_derives_the_deceleration_limit` |
| `A28` | `check_velocity_bounds` → `if joint_limit.max_velocity > joint_model.variable_bounds()[0].max_velocity` (upper end only) | `a_negative_max_velocity_below_the_models_min_is_rejected` |
| `A29` | the same, `< …min_velocity` (lower end only) | `a_max_velocity_above_the_models_is_rejected` |
| `A30` | rule 4 additionally run in reverse: deceleration alone derives `max_acceleration = -max_deceleration` | `a_deceleration_override_alone_leaves_acceleration_undefined` |
| `A31` | `check_position_bounds`' `min_position` test → `joint_limit.min_position < joint_model.variable_bounds()[0].min_position` (reads the member instead of asking the model) | `a_continuous_joint_accepts_any_position_override` |
| `A32` | the same test inverted: `> …min_position` | `a_min_position_below_the_models_is_rejected`, `a_stricter_position_override_replaces_the_models_window` |
| `A33` | the guard's call-site condition → `overrides.has_limit(name)` (guard scoped to any overridden joint) | `an_acceleration_only_override_on_a_multi_variable_joint_is_allowed` |
| `A34` | the same condition drops its `|| joint_limit.has_velocity_limits` disjunct | `a_velocity_override_on_a_multi_variable_joint_is_rejected` |
| `A35` | the same condition drops its `joint_limit.has_position_limits ||` disjunct | `a_position_override_on_a_multi_variable_joint_is_rejected` |
| `A36` | rule 4: `= -max_acceleration` → `= -max_acceleration.min(2.0)` | `an_acceleration_override_alone_derives_the_deceleration_limit` |
| `A37` | `check_position_bounds`' `max_position` test with margin `-1e-9` | `a_position_override_equal_to_the_models_bounds_is_accepted` |
| `A38` | the `min_position` test additionally rejects an override strictly inside the model's window | `a_stricter_position_override_replaces_the_models_window` |

Three of these are worth reading against each other rather than one at a
time:

- `A1`/`A2` bracket the multi-DOF guard from both sides: too loose and the
  two rejection tests stop rejecting, too tight and nine ordinary
  single-variable cases start rejecting. `A34`/`A35` then split the guard's
  two disjuncts, so each rejection test is attributable to *its own*
  dimension, and `A33` shows the guard is scoped to the checks rather than
  to the joint — an acceleration-only override on a planar joint must still
  pass.
- `A28`/`A29` split `satisfiesVelocityBounds`' two ends. The port's single
  call tests both, so without these two the pair of velocity-rejection
  tests would only ever fail together (`A6`).
- `A24`/`A37`/`A38` split the three position cases that all assert "a
  checked override survives unchanged": the equal-to-bounds one dies to a
  negative margin on *either* comparison, the strictly-inside one to an
  added strictly-inside rejection, and the continuous one to `A31`, which
  fires only because a continuous joint's `satisfies_position_bounds` is
  unconditionally `true` while its stored `min_position` is not `-1000`.

## 3. `joint_limits_validator.rs` — 17 tests, 16 exclusive mutations

This module produces **zero** scanner sites; the table exists because the
guards exist, not because the scanner asked.

| id | mutation | fails |
|---|---|---|
| `V1` | `validate_with`: the `None` arm → `false` | `an_empty_container_agrees_on_every_dimension` |
| `V2` | `validate_with`: compare only the first pair | `a_third_joint_disagreeing_with_the_first_two_is_still_a_disagreement` |
| `V3` | `position_equal`: the `has_position_limits` inequality check dropped | `position_flags_that_differ_are_a_disagreement` |
| `V4` | `position_equal`: the `max_position` comparison always agrees | `a_differing_max_position_is_a_disagreement`, `a_third_joint_disagreeing_with_the_first_two_is_still_a_disagreement` |
| `V5` | `position_equal`: the `min_position` comparison always agrees | `a_differing_min_position_is_a_disagreement` |
| `V6` | `position_equal`: values compared even when the flag is clear | 10: `a_differing_max_acceleration_is_a_disagreement`, `a_differing_max_deceleration_is_a_disagreement`, `a_differing_max_velocity_is_a_disagreement`, `acceleration_flags_that_differ_are_a_disagreement`, `acceleration_values_behind_a_clear_flag_are_not_compared`, `deceleration_flags_that_differ_are_a_disagreement`, `deceleration_values_behind_a_clear_flag_are_not_compared`, `position_values_behind_a_clear_flag_are_not_compared`, `velocity_flags_that_differ_are_a_disagreement`, `velocity_values_behind_a_clear_flag_are_not_compared` |
| `V7` | `velocity_equal`: the flag inequality check dropped | `velocity_flags_that_differ_are_a_disagreement` |
| `V8` | `velocity_equal`: the `max_velocity` comparison always agrees | `a_differing_max_velocity_is_a_disagreement`, `partially_specified_limits_are_never_equal_to_each_other` |
| `V9` | `velocity_equal`: value compared even when the flag is clear | 12 |
| `V10` | `acceleration_equal`: the flag inequality check dropped | `acceleration_flags_that_differ_are_a_disagreement` |
| `V11` | `acceleration_equal`: the value comparison always agrees | `a_differing_max_acceleration_is_a_disagreement` |
| `V12` | `acceleration_equal`: value compared even when the flag is clear | 13 |
| `V13` | `deceleration_equal`: the flag inequality check dropped | `deceleration_flags_that_differ_are_a_disagreement` |
| `V14` | `deceleration_equal`: the value comparison always agrees | `a_differing_max_deceleration_is_a_disagreement` |
| `V15` | `deceleration_equal`: value compared even when the flag is clear | `deceleration_values_behind_a_clear_flag_are_not_compared` |
| `V16` | `validate_with`: a container with nothing to compare against answers `false` | `a_single_joint_agrees_with_itself_on_every_dimension` |
| `V17` | `velocity_equal`: two `NaN`s behind a *set* flag compare equal | `partially_specified_limits_are_never_equal_to_each_other` |
| `V18` | `velocity_equal`: values within `2.0` of each other compare equal, `NaN` still unequal | `a_differing_max_velocity_is_a_disagreement` |
| `V19` | `position_equal`: values compared behind a clear flag *only when both are non-`NaN`* | `position_values_behind_a_clear_flag_are_not_compared` |
| `V20` | `velocity_equal`: the same restriction | `velocity_values_behind_a_clear_flag_are_not_compared` |
| `V21` | `acceleration_equal`: the same restriction | `acceleration_values_behind_a_clear_flag_are_not_compared` |

`V6`, `V9` and `V12` are the reason `V19`–`V21` had to be written. The
obvious mutation for "values behind a clear flag are not compared" — drop
the flag gate outright — takes down 10, 12 and 13 tests respectively, not
because those tests exercise the guard but because every other fixture
leaves the *other* three dimensions at `JointLimit::default()`, whose
numbers are `NaN`, and `NaN != NaN`. So dropping any one dimension's flag
gate makes that dimension disagree in nearly every fixture at once. Adding
`… || (both values are non-NaN)` narrows the mutation to fixtures that
actually carry numbers behind a clear flag, which is exactly one test each.

`V15` needs no such treatment: `JointLimit::default()`'s `max_deceleration`
is `0.0`, not `NaN`, so the deceleration dimension already agrees across
the other fixtures and the naive mutation is isolating on the first try.

`V17`/`V18` split the two claims `V8` collapses: that a differing
`max_velocity` disagrees, and that two *identical* partially-specified
limits also disagree because the value behind the set flag is `NaN`.

## 4. The one test with no exclusive mutation

`a_differing_max_position_is_a_disagreement` (two joints, `max_position`
`1.0` vs `2.0`) is a strict sub-case of
`a_third_joint_disagreeing_with_the_first_two_is_still_a_disagreement`
(three joints, `max_position` `1.0`, `1.0`, `2.0`). Any mutation of the
`max_position` comparison changes both verdicts identically, and mutations
that separate them by *magnitude* cannot exist either — both fixtures use
the same `1.0` difference. Three were tried and all took the pair together:
neutralize the comparison (`V4`), give it a tolerance of `2.0`, and make it
order-dependent (`lhs.max_position > rhs.max_position`).

The reverse separation does exist: `V2` fails the three-joint test alone,
because only it has a third element for a first-pair-only scan to skip.
So the two tests are not redundant — the smaller one isolates the
*dimension*, the larger one isolates the *scan* — but the smaller one's
coverage claim is a family of two here, not an exclusive one.

## Families, stated as families

Where a mutation took more than one test down, this ledger names all of
them above rather than crediting the first. The families are: `A1` (2),
`A2` (9), `A3` (2), `A6` (2), `A8` (2), `A13` (2), `A15` (2), `A17` (14),
`A18` (2), `A19` (2), `A20` (4), `A24` (2), `A27` (3), `A32` (2), `V4` (2),
`V6` (10), `V8` (2), `V9` (12), `V12` (13). Every one of those tests except
`a_differing_max_position_is_a_disagreement` also has a mutation that fails
it alone, listed in §2 and §3.

## Gate scope

`-p moveit-planners-pilz` for every one of the 58 mutation runs (37
aggregator, 21 validator) and for the baseline. Every revert was confirmed
byte-identical against the pre-mutation text. For the 21 validator runs
`git status --porcelain` on the mutated file was additionally empty after
the revert; the 37 aggregator runs were taken while
`joint_limits_aggregator.rs` was still untracked, so `git status` could not
answer "reverted" for them and byte equality is the whole evidence there.
The tree was clean at the end of the sweep. The full-workspace variants are
owed before `git push` under the standing rule and were not run for this
ledger.

## UNFIXED

`a_differing_max_position_is_a_disagreement` has no isolating mutation, for
the structural reason in §4. It is recorded as a family of two with
`a_third_joint_disagreeing_with_the_first_two_is_still_a_disagreement`, not
claimed as exclusive.
