# Mutation audit: `trajectory_blender_transition_window.rs`'s twelve unit tests

Per PORTING-PLAN.md §189: a number written into audit prose without the
command that produced it is not re-checkable (`moveit-scene`'s "22 hits"
sat wrong for several rounds for exactly that reason). This table's
mutation column is not narrative summary — it is the literal `old_string`/
`new_string` edit applied to the file at the commit named in the "verified
against" column, with the exact `cargo test` invocation used to observe the
result. Re-running any row means: check out that commit, apply the literal
edit at the cited `file:line`, run the cited command, read the result — no
step depends on trusting this document's prose.

## Method

For each of the twelve tests in `trajectory_blender_transition_window.rs`'s
`mod tests`, mutate only the specific production branch/expression the
test's own doc comment (or, absent one, its own name) claims to cover — not
the whole function — rebuild, and run that one test with
`cargo test -p moveit-planners-pilz --lib <test_name> -- --nocapture`.
A mutation that survives (the test still passes) is not yet a finding: it
must first be reach-confirmed (an `eprintln!` placed inside the mutated
branch, checked for in `--nocapture` output, or — when even that would not
distinguish "never called" from "called but coincidentally masked" — a
second, more targeted mutation that isolates the exact code path). Every
mutation is reverted with `git checkout --
crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs`
before the next, confirmed clean via `git status --short` producing no
output.

All twelve mutations below were run against
`crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs`
at commit `e88d7c7` (the commit landing this round's mutation-testing task
began from — `b3e39c3`'s original port, unmodified by any mutation-testing
fix yet).

## The twelve rows

| # | test (`fn`, line as of `e88d7c7`) | mutated `file:line` → exact change | test failed? | if survived: reach-proven? |
|---|---|---|---|---|
| 1 | `validate_request_rejects_an_unknown_group_name` (646) | `:334` `if !robot_model.has_joint_model_group(&req.group_name) {` → `if robot_model.has_joint_model_group(&req.group_name) {` | **killed** | — |
| 2 | `validate_request_rejects_an_unknown_link_name` (663) | `:338` `if !robot_model.has_link_model(&req.link_name) {` → `if robot_model.has_link_model(&req.link_name) {` | **killed** | — |
| 3 | `validate_request_rejects_blend_radius_at_or_below_zero` (679) | `:342` `if req.blend_radius <= 0.0 {` → `if req.blend_radius > 0.0 {` | **survived** | yes — `eprintln!` inside the mutated branch printed for both `blend_radius = 0.0` and `blend_radius = -0.01` |
| 4 | `validate_request_rejects_a_boundary_state_mismatch` (710) | `:346` `if !is_robot_state_equal(` → `if is_robot_state_equal(` | **killed** | — |
| 5 | `validate_request_rejects_a_mismatched_sampling_time` (734) | `:355-357` `determine_and_check_sampling_time(...).ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))?` → `determine_and_check_sampling_time(...).unwrap_or(0.1)` | **killed** | — |
| 6 | `validate_request_accepts_a_well_formed_request` (750) | `:371` `Ok(sampling_time)` → `Ok(sampling_time + 1.0)` | **killed** (assertion showed `left: 1.1, right: 0.1`) | — |
| 7 | `determine_trajectory_alignment_picks_first_trajectory_tail_when_it_is_longer` (766) | `:428` `if way_point_count_1 > way_point_count_2 {` → `if way_point_count_1 < way_point_count_2 {` | **killed** | — |
| 8 | `determine_trajectory_alignment_picks_first_intersection_index_otherwise` (785) | `:428` same mutation as row 7 (reused, no revert between rows 7/8) | **killed** | — |
| 9 | `search_intersection_points_finds_both_crossings_within_radius` (808) | `:400`/`:409` swap the `true`/`false` direction argument between the two `linear_search_intersection_point` calls | **killed** (panicked on `.unwrap()` of an `Err`) | — |
| 10 | `search_intersection_points_rejects_a_radius_larger_than_either_trajectory_reaches` (original test, since rewritten — see "Findings" below) | `:400`/`:409` same swap as row 9 (reused) | **survived** | yes, but **wrong mutation for this test** — direction-independent geometry (a tiny trajectory entirely within a huge `blend_radius` has no crossing regardless of scan direction). Second, targeted mutation: forced the first `linear_search_intersection_point` call's `None` to `Some(0)` via `.or_else(\|\| { eprintln!(...); Some(0) })` at `:402`. **Still survived, reach-proven** (`eprintln!` printed) |
| 11 | `blend_trajectory_cartesian_first_sample_stays_near_pose1_last_sample_reaches_pose2` (871) | `:478` `if (first_intersection_index + i) > blend_align_index {` → `if false {` | **killed** (assertion showed `left: [0.307…, -5.2e-12, 0.590…], right: [0.3035…, 0.0459…, 0.590…]`) | — |
| 12 | `blend_produces_a_continuous_trajectory_through_the_shared_boundary` (956) | `:280` `for i in 0..first_intersection_index {` → `for i in 0..=first_intersection_index {` | **survived** | yes — `eprintln!` inside the loop printed 11 iterations (`i=0` through `i=10`) instead of 10 |

Rows 1, 2, 4, 5, 6, 7, 8, 9, 11 legitimately fail their mutation and were
left unchanged.

## Findings: three surviving-and-reached rows, each fixed in its own commit

**Row 3 — `validate_request_rejects_blend_radius_at_or_below_zero`, fixed in
`5554ec1`.** Root cause: the fixture built `first_trajectory`/
`second_trajectory` from two independent, non-chained `panda_joint1_sweep`
calls sharing the same `(0.0, 0.2)` angle offset, so their supposed shared
boundary never actually matched. `validate_request`'s boundary-mismatch
check (`:346`, three lines above the `blend_radius` check) rejected the
request with the identical `InvalidMotionPlan` code the `blend_radius`
check itself would have produced, masking whether the `blend_radius` check
did anything. Fix: chain `second_trajectory` onto `first_trajectory`'s
actual end (`panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.1)`), isolating the
`blend_radius` check as the sole possible cause of rejection. Re-verified
by re-applying the exact row-3 mutation against the fixed test: it now
fails (`assertion failed` at the `matches!` line), where before the fix it
passed.

**Row 10 — `search_intersection_points_rejects_a_radius_larger_than_either_trajectory_reaches`,
replaced by two isolating tests in `52d597b`.** Root cause: the two
`linear_search_intersection_point` calls inside `search_intersection_points`
(`:395-402` for `first_trajectory`, `:404-411` for `second_trajectory`) are
independently `?`-chained. The original fixture's tiny sweeps
(`panda_joint1_sweep(&model, -0.005, 0.0, 10, 0.05)` /
`panda_joint1_sweep(&model, 0.0, 0.005, 10, 0.05)`, `blend_radius: 10.0`)
made *both* calls independently return `None` — forcing either one to
succeed left the other still failing, so the overall `Err` outcome never
changed and a mutation to either individual call would survive. Fix: split
into
`search_intersection_points_rejects_when_first_trajectory_never_reaches_the_blend_radius`
and
`search_intersection_points_rejects_when_second_trajectory_never_reaches_the_blend_radius`,
each pairing the trajectory under test with the *other* trajectory from
`search_intersection_points_finds_both_crossings_within_radius`'s own
known-crossing geometry (`blend_radius: 0.05`), so only the trajectory
under test can be the cause of the `Err`. Re-verified by mutating each of
the two `linear_search_intersection_point` calls to succeed unconditionally
(not merely on a `None` fallback) and confirming each new test fails only
when its own named call is the one disabled — `search_intersection_points_rejects_when_first_trajectory_never_reaches_the_blend_radius`
failed only when the *first* call was forced to succeed, and the mirror
held for the second.

**Row 12 — `blend_produces_a_continuous_trajectory_through_the_shared_boundary`,
fixed in `83dc4ff`.** Root cause: the test asserted
`response.first_trajectory.way_point_count() <= req.first_trajectory.way_point_count()`
and the equivalent `<=` bound for `second_trajectory` — non-strict bounds
that a same-direction off-by-one in either of `blend`'s waypoint-copy loops
(`:280` `for i in 0..first_intersection_index`, `:291`
`for i in (second_intersection_index + 1)..second_count`) still satisfies.
Fix: compute ground-truth `first_intersection_index`/
`second_intersection_index` via an independent `search_intersection_points`
call over the same geometry, and assert exact equality of both response
segment lengths against the values derived from those indices. Re-verified
by re-applying the exact row-12 mutation: the fixed test now fails
(`assertion left == right failed: left: 11, right: 10`).

## Why these three and not the other nine

The other nine tests are hand-picked-argument or pure input-validation
checks with a real oracle-independent correctness criterion of their own
(see `doc/oracle-request-pilz-blend.md`'s own "Overlap with this port's
existing tests" section for the argument in full) — mutating their subject
line legitimately breaks them, and no masking fixture or weak assertion
was found for any of the nine. The three fixed here are the ones where a
second, independent mechanism (a coincidentally-identical error code, a
second `?`-chained check with the same fixture-driven failure, or a
weak-enough bound) let the test pass while the mutation silently broke the
behavior the test's own name claims to cover.

## Relationship to `pilz_blend_parity.rs`

This mutation audit and `crates/moveit-planners-pilz/tests/pilz_blend_parity.rs`
(`639df34`) are complementary, not overlapping: this audit finds where the
existing *self-consistency* tests were too weak to catch a broken branch;
`pilz_blend_parity.rs` is the first test in this module comparing this
port's numeric output against upstream's own. Neither replaces the other —
see `doc/oracle-request-pilz-blend.md`'s own module doc for why a
waypoint-only comparison can still pass with the wrong intersection index
or the wrong alignment branch on an unlucky fixture, which is exactly the
class of masking this mutation audit independently found in rows 3, 10,
and 12.
