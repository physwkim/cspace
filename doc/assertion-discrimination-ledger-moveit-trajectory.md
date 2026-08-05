# Assertion-discrimination ledger — `moveit-trajectory`

Produced by p1-joints, assertion-discrimination-round2. Fence:
`crates/moveit-trajectory`. Enumerated with
`tools/ci/count-coarse-assertions.py crates/moveit-trajectory` post-merge
(`6792ef1`, `f10e1bd`, `ccac7ea` — helper-body scoping, collapsed
`contains` kind, top-level-operand `eq_none`/`eq_err`). Raw scan at that
time: 56 sites; 40 were tabled, and 16 bare `.is_err()`/`.is_none()`
sites were left uncited without a documented reason — that gap
surfaced this round as 16 of 24 "orphans" (p3-acm's closing audit
against `tools/ci/count-coarse-assertions.py`), the other 8 being
merge-driven line drift (7, this file, from sibling tests other panels
inserted earlier in it) plus this round's own new test (`:1561`) not
yet having a table row. All 24 are now resolved below (13 new
single-branch/discriminating rows, 7 citations corrected for drift,
1 new discriminating row, 1 reclassified not-this-family in
`ruckig_smoothing.rs`); current raw scan (this branch, including this
round's own new test) is **57**, all tabled:

| file | sites |
|---|---:|
| `src/robot_trajectory.rs` | 9 |
| `src/time_optimal_trajectory_generation.rs` | 13 |
| `src/path.rs` | 5 |
| `src/trajectory.rs` | 4 |
| `tests/robot_trajectory.rs` | 22 |
| `tests/ruckig_smoothing.rs` | 4 |
| **total** | **57** |

Verdict legend: **discriminating** (bite-confirmed: neutralizing the
guard changes the targeted test's outcome, a sibling test/fixture stays
green), **single-branch** (exactly one possible `Error`/`None`
construction site reaches this assertion — confirmed by reading the
callee's full body and finding exactly one `return Err`/`ok_or_else`;
nothing to discriminate, so no bite is needed or possible), **not-this-family**
(fails census §9 clause 1 — inspects a
plain success-path `Display`/`Debug`-formatted value, not a genuine
failure/absence signal), **structurally-redundant-operand** (in-family,
overall discriminating, but one operand of a folded `||`/`&&` guard is
bite-confirmed dead — no fixture can ever isolate it, so it is not a
fixable "blind assertion"; flagged for the user's call on whether to
simplify the guard), **fixed** (was blind — a bite showed the old
assertion's outcome was unaffected by the real defect it claimed to
guard against; a new assertion was written and bite-confirmed this
round), **uncovered, no verdict** (a written guard with zero assertion
anywhere in the crate touching it; per
`doc/folded-operand-guards.md`, noted, not fabricated a verdict for, not
fixed).

Evidence legend: **bite** (a fresh isolating mutation this round, run
with `--no-fail-fast`, reverted after confirming — `cond && !true` to
neutralize a guard live, `cond || true` to force it, keeping all
operands live per this round's compile-under-`-D warnings` constraint).
Two bites hung rather than failed cleanly (`trajectory.rs`'s two
`Trajectory::create` guards) — killed via `timeout`/`TaskStop` and
treated as stronger-than-clean-failure proof of load-bearing-ness, not
as inconclusive.

## `path.rs` (5)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `path.rs:217` | `Path::create`'s `waypoints.len() < 2` guard, 1-waypoint fixture | `fewer_than_two_waypoints_is_rejected` | discriminating | bite (`&& !true` on the guard; fails both assertions, siblings green) |
| `path.rs:223` | same guard, 0-waypoint fixture | same fn | discriminating | same bite |
| `path.rs:236` | `Path::create`'s `max_deviation <= 0.0` guard, zero fixture | `zero_max_deviation_is_rejected` | discriminating | bite (independent of the waypoint-count guard; message-unique) |
| `path.rs:242` | same guard, negative fixture | same fn | discriminating | same bite |
| `path.rs:318` | blend loop's `cos_angle <= -1.0 + ANGLE_TOLERANCE` guard (180° turn) | `a_180_degree_turn_is_rejected` | discriminating | bite (message-unique against the other two guards) |

## `robot_trajectory.rs` (src, 9)

All 9 sites check `Display`-formatted `to_string()` output of a
successfully-built trajectory for column presence/absence
(`" pos "`, `" vel "`, etc.) — a computed, informative success-path
value, not a failure/absence signal. Fails census §9 clause 1.

| file:line | anchor | test fn | verdict |
|---|---|---|---|
| `robot_trajectory.rs:836` | printed column check | `empty_trajectory_prints_the_upstream_placeholder` | not-this-family (clause 1) |
| `robot_trajectory.rs:839` | same | same fn | not-this-family (clause 1) |
| `robot_trajectory.rs:840` | same | same fn | not-this-family (clause 1) |
| `robot_trajectory.rs:841` | same | same fn | not-this-family (clause 1) |
| `robot_trajectory.rs:846` | same | `position_only_waypoints_omit_the_conditional_columns_and_use_the_group_variables` | not-this-family (clause 1) |
| `robot_trajectory.rs:876` | same | `velocity_acceleration_and_effort_columns_appear_when_the_waypoint_carries_them` | not-this-family (clause 1) |
| `robot_trajectory.rs:877` | same | same fn | not-this-family (clause 1) |
| `robot_trajectory.rs:878` | same | same fn | not-this-family (clause 1) |
| `robot_trajectory.rs:879` | same | same fn | not-this-family (clause 1) |

## `time_optimal_trajectory_generation.rs` (7)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `time_optimal_trajectory_generation.rs:1043` | `with_resample_dt`'s single `!is_finite() \|\| <= 0.0` guard, `resample_dt = 0.0` | `resample_dt_zero_is_rejected_not_hung` | single-branch | structural: `with_resample_dt` has exactly one `if`/one `return Err` in its body |
| `time_optimal_trajectory_generation.rs:1056` | same guard, `resample_dt = -0.01` | `resample_dt_negative_is_rejected_not_silently_truncated` | single-branch | same |
| `time_optimal_trajectory_generation.rs:1083` | `do_time_parameterization_calculations`'s sample-count guard, `!is_finite() \|\| > MAX` fold, tiny-`resample_dt` fixture (cited `:1070` before merge-driven line drift) | `resample_dt_producing_an_unreasonable_sample_count_is_rejected` | discriminating; both operands live | superseded — see "Correction" below |
| `time_optimal_trajectory_generation.rs:1110` | same guard, subnormal fixture (cited `:1097` before drift) | `resample_dt_subnormal_is_rejected` | same | same |
| `time_optimal_trajectory_generation.rs:1125` | `with_resample_dt`'s single guard, `resample_dt = NaN` | `resample_dt_nan_is_rejected` | single-branch | structural, same as `:1043` |
| `time_optimal_trajectory_generation.rs:1138` | same guard, `resample_dt = +inf` | `resample_dt_positive_infinity_is_rejected` | single-branch | same |
| `time_optimal_trajectory_generation.rs:1149` | same guard, `resample_dt = -inf` | `resample_dt_negative_infinity_is_rejected` | single-branch | same |
| `time_optimal_trajectory_generation.rs:1176` | same fold as `:1078`, usize-max-boundary fixture (cited `:1163` before drift) | `resample_dt_targeting_the_usize_max_boundary_is_rejected` | discriminating; both operands live | superseded — see "Correction" below |
| `time_optimal_trajectory_generation.rs:1434` | `do_time_parameterization_calculations`'s `max_velocity.len() != num_joints \|\| max_acceleration.len() != num_joints` fold (line 770), mimic-joint-group fixture (4 active vs 7 full variables); cited `:1421` before drift | `mimic_joint_group_is_a_typed_error_not_a_panic` | discriminating; `max_acceleration.len() != num_joints` operand deleted, dead by construction | **fixed this round** (`612a9b3`) — see "Correction" below |
| `time_optimal_trajectory_generation.rs:1495` | `compute_time_stamps_with_limits`'s custom-limit-vs-bounds-fallback branch (the `acceleration_set` flag at line 590); cited `:1470` before drift | `a_zero_custom_limit_skips_bound_validation` | **fixed this round** (was blind) | see below |
| `time_optimal_trajectory_generation.rs:1566` | `do_time_parameterization_calculations`'s sample-count guard, `!is_finite()` operand isolated, custom-`0.0`-velocity-limit fixture | `resample_dt_over_a_nan_duration_is_rejected` | discriminating (new this round) | bite: `!is_finite()` alone neutralized → silent `Ok(())` with NaN `sample_count` saturating to `0`; see "Correction" above. Commit `b12b358` |
| `time_optimal_trajectory_generation.rs:1592` | `totg_compute_time_stamps`'s `num_waypoints < 2` guard, `num_waypoints = 1` fixture; cited `:1511` before drift | `totg_compute_time_stamps_rejects_fewer_than_two_waypoints` | discriminating | bite (`&& !true`; both assertions in the fn fail cleanly, single guard in the function) |
| `time_optimal_trajectory_generation.rs:1598` | same guard, `num_waypoints = 0` fixture; cited `:1517` before drift | same fn | discriminating | same bite |

**Correction — `:1070`/`:1097`/`:1163` and `:1421` (this round).** The
prior round's verdicts above ("structurally-redundant") were reached by
reading the constructor, not by a reachability test — exactly the census
§9g funnel shape. Re-investigated on request:

- `:1421`'s fold (`max_velocity.len() != num_joints || max_acceleration.len()
  != num_joints`, source `:770`): confirmed dead **by construction**.
  `do_time_parameterization_calculations` is a private `fn` with exactly
  two call sites in this file (`compute_time_stamps`,
  `compute_time_stamps_with_limits`), and both construct
  `max_velocity`/`max_acceleration` as `DVector::zeros(num_active)` from
  the *same* `num_active` binding for both vectors — the two lengths can
  never independently differ. **Fixed by deletion** (`612a9b3`): the
  `max_acceleration.len() != num_joints` operand is gone. Bite-reconfirmed
  the surviving `max_velocity.len() != num_joints` operand is still
  load-bearing (neutralizing it alone now fails
  `mimic_joint_group_is_a_typed_error_not_a_panic` with a panic, not a
  clean error).
- `:1070`/`:1097`/`:1163`'s `!is_finite()` operand: **not** provably dead.
  One NaN-producing mechanism (zero-length-path collapse) is ruled out by
  construction via the diversity-collapse loop's push/replace invariant,
  but a second, distinct mechanism — a moving joint carrying a custom
  `0.0` velocity limit — was **empirically reproduced** through the real
  public API (`compute_time_stamps_with_limits`, panda_arm, 1e-5-scale
  path, `min_angle_change: 0.0`) and produces a NaN `duration()`, which
  `> MAX_RESAMPLE_SAMPLE_COUNT` does **not** catch (`NaN > x` is always
  `false`). This was a coverage gap, not dead code — per the user's own
  rule, the answer is a test, not a deletion. **Fixed by adding**
  `resample_dt_over_a_nan_duration_is_rejected` (commit `b12b358`);
  bite-confirmed neutralizing `!is_finite()` alone turns this fixture into
  a silent `Ok(())` (NaN `sample_count` saturates to `0` under `as
  usize`) instead of the resample-bound error the rest of this guard's
  family gets.

This NaN/`+inf` behaviour is itself a faithfully-transcribed upstream bug
(`time_optimal_trajectory_generation.cpp:405`, no zero-relative-velocity
guard on the timing-loop division) — recorded as
`doc/upstream-bugs.md`'s `totg-timing-zero-velocity-division`,
`reproduced-grandfathered` per the user's 2026-08-05 decision (document
only, code unchanged), not fixed here.

**Fix — `a_zero_custom_limit_skips_bound_validation` (commit `8ca3c3f`).**
The old assertion was `!message.contains("invalid max_acceleration")` —
checking the *absence* of the model-bounds-fallback branch's own error
text. `panda()`'s URDF-loaded joint bounds never set
`acceleration_bounded` (confirmed by reading the loader and by this
crate's own `totg_compute_time_stamps_silently_collapses_duplicate_
waypoints_matching_upstream`, which has to hand-set it via
`joint_model_mut` to reach that branch at all elsewhere in this file).
That means the bounds-fallback branch is **structurally unreachable**
for this test's fixture regardless of whether the custom-limit-applied
flag (`acceleration_set = true` at line 590) is implemented correctly.
Bite: neutralizing that flag (`acceleration_set = true && !true`)
changes the actual downstream error to `"no acceleration limit was
defined for joint 'panda_joint1'..."` — a real, different failure — and
the old assertion **still passed**, since that message also doesn't
contain "invalid max_acceleration". Fixed by asserting on
`Trajectory::create`'s own distinguishing phrase (`"after
integrateForward and integrateBackward"`, matching `trajectory.rs`'s own
`DISTINGUISHING_PHRASE` constant) instead — bite-confirmed this now
fails under the same mutation.

## `trajectory.rs` (4)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `trajectory.rs:1434` | `Trajectory::create`'s post-forward-loop `!traj.valid` guard, zero-accel-on-moving-joint fixture | `upstream_test_relevant_zero_max_accelerations_invalidate_trajectory` | discriminating | bite: guard neutralized → `cargo nextest` hung, killed via `timeout 60`/`TaskStop`; treated as load-bearing confirmation, not inconclusive |
| `trajectory.rs:1440` | same guard, mirror fixture | `upstream_test_irrelevant_zero_max_accelerations_dont_invalidate_trajectory` (sibling — stays green at baseline; this row is the paired positive-message-uniqueness check) | discriminating | same |
| `trajectory.rs:1446` | same guard, `DISTINGUISHING_PHRASE` check | same fn as :1434 | discriminating | same |
| `trajectory.rs:1473` | `Trajectory::create`'s `time_step <= 0.0` guard | `upstream_test_time_step_zero_makes_trajectory_invalid` | discriminating | bite: guard neutralized → hung, killed via `timeout 20`, same treatment |

**Uncovered, still no verdict — `trajectory.rs`'s third `Error::construct`
site** ("trajectory not valid after the second integrateBackward pass",
inside `Trajectory::create`, after the second `integrate_backward` pass).
`rg` confirms this exact message string has exactly one occurrence in the
crate — its own definition; no test anywhere asserts on it, and upstream's
own test suite (`moveit2/moveit_core/trajectory_processing/test/
test_time_optimal_trajectory_generation.cpp`) has no case targeting this
branch either.

This round made three bounded, `timeout`-wrapped fixture attempts
(recorded in a prior pass of this ledger) that did not succeed:

1. A sharp near-180° corner with strong acceleration asymmetry (10.0 vs
   1e-6) — failed the *first* `!traj.valid` checkpoint instead of the
   targeted second one.
2. A gentler blend (tolerance 2.0, accel 10.0 vs 0.01) — passed both
   checkpoints (`Ok`), but produced a several-thousand-step trajectory
   (2.2MB of debug output) — an unreliable, slow fixture, not a usable
   test; abandoned rather than committed.
3. A third variant between the two, also failed to isolate the second
   checkpoint specifically.

**Reachability analysis (this round, superseding the "flag for user
decision" framing above).** Per instruction, worked as a reachability
argument rather than a fourth fixture hunt.

Control-flow parity: `moveit2 @ e017c91e`'s `Trajectory::create`
(`time_optimal_trajectory_generation.cpp:358-395`) has an *identical*
while-loop / two-checkpoint structure, including the same unconditional
second `integrateBackward(trajectory_, path_.getLength(), 0.0, ...)`
call regardless of whether the loop exited via full forward integration
or via `getNextSwitchingPoint` finding none. This branch is therefore
not a porting bug either way — whatever the true answer is, the port
did not drop or add a guard relative to upstream. Upstream's own
`RCLCPP_ERROR` for this exact case uses a distinct message from the
first checkpoint's, and upstream's own test suite
(`test_time_optimal_trajectory_generation.cpp`) — which explicitly
covers the *sibling* first-checkpoint failure
(`testRelevantZeroMaxAccelerationsInvalidateTrajectory`) — has no case
targeting this second one either, in a file exercising this algorithm's
edge cases for years.

Mechanism found: `integrate_backward`'s deceleration-branch
(`min_max_path_acceleration(pos, vel, false)`, `trajectory.rs:751-766`)
has no velocity-ceiling check, unlike `integrate_forward`, which checks
`path_vel > acceleration_max_path_velocity(path_pos) ||
path_vel > velocity_max_path_velocity(path_pos)` on *every* step
(`trajectory.rs:595-596`, not only at detected switching points) and
bisection-corrects immediately, routing the correction through the
*first* `integrate_backward` call site instead. For the *second* call
to hit `path_vel < 0.0` (`:711-714`) or exhaust the loop without an
intersection (`:745`), the backward-from-rest walk would need to
compute a velocity exceeding the local ceiling at some point the
forward pass's continuous check never touched.

Two additional, mechanism-targeted probes this round tried to force
exactly that gap open — not a repeat of the prior three's blind
fixture search:

4. A near-180° corner (radius ≈1.2e-3) placed shortly before a short
   (~0.25-unit) straight tail at the end of the path, high `max_velocity`
   relative to `max_acceleration` to force speed to build in the long
   leg before the corner: `Ok`, 9376 steps.
5. The same corner shape but with the final waypoint placed *inside*
   the blend distance (`end_distance < deviation cap`, so the blend
   consumes the entire final segment and `path.length()` lands exactly
   on the arc's exit — zero straight run-out, curvature nonzero from
   the very first backward step): `Ok`, 9101 steps.

Both succeeded rather than failed: `next_switching_point`'s scan
(`:325-376`) still located and pre-handled the corner via the *first*
`integrate_backward` call site regardless of its proximity to
`path.length()`, and forward integration's own continuous ceiling
check (`:595-596`) still bisected the approach down to a decelerable
velocity before ever leaving the arc behind at an unrecoverable speed —
consistent with the mechanism above, i.e. it is evidence *for* the
"forward integration's per-step ceiling check protects this branch"
hypothesis, not proof of it.

**Verdict: cannot be determined further without disproportionate
effort.** Specific blocker: confirming unreachability requires either
(a) a formal argument that `integrate_forward`'s continuous per-step
ceiling check is *sufficient* — for every path/limits combination, not
just the two probed — to guarantee the accumulated trajectory is
decelerable-to-rest everywhere, which this round's analysis sketches
but does not complete, or (b) a substantially larger numerical search
over path/limit parameter space than five targeted probes (three
fixture-shaped, two mechanism-shaped) have covered. Both probe scratch
tests were reverted, not committed (`git diff --stat` empty against the
tree before this update) — an always-`Ok` probe would be a test that
passes for a reason unrelated to the branch. The branch is **not**
removed: it is not proven dead, upstream retains the structurally
identical branch with its own distinct diagnostic, and if it ever is
proven unreachable, the same argument applies to upstream's copy, not
just the port's — that determination is out of this round's scope.
Remains: a written guard with zero covering assertion anywhere in the
crate, UNFIXED.

## `tests/robot_trajectory.rs` (12)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `tests/robot_trajectory.rs:65` | `is_empty()` on a freshly-built trajectory (test setup state check, not a subject decision) | `init_test_trajectory`-adjacent helper assertion | not-this-family (clause 1 — plain success-path state, not a failure/absence signal from a decision under test) | — |
| `tests/robot_trajectory.rs:310` | `smoothness`'s `waypoints.len() <= 2` guard (src `:761`), positive-value fixture | `smoothness_is_some_positive_value_and_none_when_empty` | discriminating | bite (`&& !true`; target fn panics with out-of-bounds access — the guard was masking a real precondition — sibling `waypoint_density` test stays green) |
| `tests/robot_trajectory.rs:314` | same guard, post-`clear()` fixture | same fn | discriminating | same bite |
| `tests/robot_trajectory.rs:321` | `waypoint_density`'s `length > 0.0` guard (src `:789`), 5-identical-waypoints (zero geometric length) fixture | `waypoint_density_is_none_at_zero_length_and_some_after_perturbation` | discriminating | bite (`\|\| true` forces the guard true → `Some(inf)` returned, assertion fails; sibling `smoothness` test stays green) |
| `tests/robot_trajectory.rs:325` | same fn, post-perturbation positive-density fixture | same fn | discriminating | same bite |
| `tests/robot_trajectory.rs:329` | `waypoint_density`'s `waypoints.len() <= 1` guard (src `:785`), post-`clear()` (0-waypoint) fixture | same fn | discriminating overall; guard is **structurally-redundant** with `length > 0.0` | bite: guard neutralized alone (`&& !true`) → all tests stay green (dead). Structural argument: `path_length()`'s accumulation loop (`for i in 1..len`) cannot execute for `len <= 1`, so `length` is provably always exactly `0.0` there, and the `length > 0.0` guard immediately below independently returns `None` for the same fixture |
| `tests/robot_trajectory.rs:484` | `add_suffix_way_point`'s single `is_empty() && dt != 0.0` guard (src `:249`) | `add_suffix_way_point_on_an_empty_trajectory_rejects_a_nonzero_dt` | single-branch | structural: `add_suffix_way_point`'s body has exactly one `if`/one `return Err` |
| `tests/robot_trajectory.rs:514` | `insert_way_point`'s `index == 0 && dt != 0.0` guard (src, `first_duration_error`) | `insert_way_point_at_zero_rejects_a_nonzero_dt` | discriminating | bite (`&& !true`; message-unique against the sibling `index_error`) |
| `tests/robot_trajectory.rs:532` | `set_way_point_duration_from_previous`'s single `index == 0 && value != 0.0` guard (src `:234`) | `set_way_point_duration_from_previous_at_zero_rejects_a_nonzero_value` | single-branch | structural: exactly one `if`/one `return Err` in the function body |
| `tests/robot_trajectory.rs:550` | `append`'s single `self.waypoints.is_empty() && dt != 0.0` guard (src `:338`) | `append_onto_an_empty_trajectory_rejects_a_nonzero_dt` | single-branch | structural: `append`'s only other early return (`start_index >= end_index`) is `Ok`, not `Err` — one `Err` site total |
| `tests/robot_trajectory.rs:568` | `way_point`'s single `ok_or_else(index_error)` (src `:171`) | `empty_trajectory_accessors_return_typed_errors_not_panics` | single-branch | structural: one `ok_or_else` in the function body |
| `tests/robot_trajectory.rs:569` | `first_way_point`'s single `ok_or_else(empty_error)` (src `:186`) | same fn | single-branch | same |
| `tests/robot_trajectory.rs:570` | `last_way_point`'s single `ok_or_else(empty_error)` (src `:196`) | same fn | single-branch | same |
| `tests/robot_trajectory.rs:590` | `is_empty()` on an emptied trajectory | `reverse_on_an_empty_trajectory_is_a_no_op` | not-this-family (clause 1) | — |
| `tests/robot_trajectory.rs:668` | `way_point(len)`'s single `ok_or_else(index_error)` | `out_of_range_index_access_is_a_typed_error` | single-branch | same structural argument as `:568` |
| `tests/robot_trajectory.rs:669` | `way_point_mut(len)`'s single `ok_or_else(index_error)` (src `:178`) | same fn | single-branch | structural: one `ok_or_else` in the function body |
| `tests/robot_trajectory.rs:670` | `remove_way_point(len)`'s single `index >= len` guard (src `:314`) | same fn | single-branch | structural: exactly one `if`/one `return Err` |
| `tests/robot_trajectory.rs:676` | `insert_way_point`'s `index > self.waypoints.len()` guard (src, `index_error`) | `out_of_range_index_access_is_a_typed_error` | discriminating | bite (`&& !true`; message-unique against `first_duration_error`) |
| `tests/robot_trajectory.rs:708` | `Debug`-formatted `"dirty: None"` substring on a waypoint after settling | `add_suffix_way_point_settles_the_stored_waypoint` | not-this-family (clause 1 — `Debug` text on a successfully-settled value) | — |
| `tests/robot_trajectory.rs:723` | same | `add_prefix_way_point_settles_the_stored_waypoint` | not-this-family (clause 1) | — |
| `tests/robot_trajectory.rs:738` | same | `insert_way_point_settles_the_stored_waypoint` | not-this-family (clause 1) | — |
| `tests/robot_trajectory.rs:749` | `for_group_name`'s single fallible call (`joint_model_group(group)?`, src `:126`) | `unknown_group_name_is_a_typed_error_not_a_silent_whole_robot_fallback` | single-branch | structural: exactly one `?`-propagated call in the function body; the external callee (`moveit-model`) is out of this crate's fence |

## `tests/ruckig_smoothing.rs` (4)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `tests/ruckig_smoothing.rs:203` | `trajectory.group().is_none()` — fixture precondition check (confirms no group was set), not the guard under test | `no_group_set_is_an_error` | not-this-family (clause 1 — plain success-path state, not a failure/absence signal from a decision under test) | — |
| `tests/ruckig_smoothing.rs:208` | `validate_group`'s single `.ok_or_else` guard (src `:245-249`) | `no_group_set_is_an_error` | discriminating | bite: message text mutated (`Error::other("mutated")`) → target fails, sibling `empty_trajectory_with_a_group_is_a_no_op` (never reaches this guard) stays green |
| `tests/ruckig_smoothing.rs:220` | `is_empty()` on a no-op-smoothed empty trajectory | `empty_trajectory_with_a_group_is_a_no_op` | not-this-family (clause 1) | — |
| `tests/ruckig_smoothing.rs:223` | same | same fn | not-this-family (clause 1) | — |

## Summary

- 57/57 sites classified against census §9 (40 from the original sweep,
  17 closed this round: 16 previously-uncited sites plus this round's own
  new test).
- Orphan reconciliation (p3-acm's closing audit, this crate's assigned
  share): re-derived independently rather than taken on the brief's word
  per instruction. `tools/ci/count-coarse-assertions.py crates/
  moveit-trajectory` against this branch gives **57** raw sites (the
  brief's premise of 56 was measured on `main` before this round's
  `b12b358` landed here — both are correct, for different commits).
  Diffing against this ledger's citations before this round's edits gave
  **24** raw mismatches, not the brief's 30: 7 were pure line drift (this
  file's `resample_dt_*`/`mimic_joint_group_*`/`a_zero_custom_limit_*`/
  `totg_compute_time_stamps_*` rows — sibling tests another panel
  inserted earlier in the file shifted every assert below them by 8-76
  lines; same test, same assert, corrected citation only, no new verdict
  needed), 1 was this round's own new test not yet tabled (`:1561`), and
  16 were genuine gaps — assertions the original 40-site sweep never
  gave a row, mostly single-branch `.is_err()` checks in small
  single-purpose test functions. All 16 are single-branch by
  construction (each callee's body has exactly one `Error`/`None`
  construction site) except `tests/ruckig_smoothing.rs:203`, which is a
  fixture-precondition check, not-this-family. None needed a bite: per
  this round's standard, a bite is required to call a funnel/multi-branch
  site discriminating, but these are not funnels — there is nothing to
  isolate from when only one cause exists.
- 2 sites were blind and are **fixed** this round: `8ca3c3f`
  (`time_optimal_trajectory_generation.rs:1475`, prior round) and
  `:1421`'s fold at source `:770` — `max_acceleration.len() !=
  num_joints` deleted, proven dead by construction, `612a9b3` (this
  round).
- 2 sites were blind and are **fixed** this round: `8ca3c3f`
  (`time_optimal_trajectory_generation.rs:1475`, prior round) and
  `:1421`'s fold at source `:770` — `max_acceleration.len() !=
  num_joints` deleted, proven dead by construction, `612a9b3` (this
  round).
- `:1070`/`:1097`/`:1163`'s `!is_finite()` operand was previously
  misclassified as structurally-dead by a read-only funnel-shape verdict.
  Re-investigated this round: one NaN mechanism is dead by construction,
  a second is live and was previously uncovered — **fixed by adding a
  test** (`resample_dt_over_a_nan_duration_is_rejected`, `b12b358`), not
  by deletion. See "Correction" above.
- `tests/robot_trajectory.rs:329`'s `waypoints.len() <= 1` guard retains
  its prior structurally-redundant verdict — **not** re-investigated this
  round (not named in this round's request; the funnel-shape correction
  applies to it too and it has not been re-verified against the current
  by-construction-or-test standard).
- 1 site remains a written guard with **no covering assertion anywhere in
  the crate** (`trajectory.rs`'s "after the second integrateBackward
  pass"). Worked this round as a reachability question, not a fixture
  hunt: control flow is identical to upstream (not a porting bug either
  way), a concrete mechanism was identified (`integrate_backward`'s
  deceleration branch has no velocity-ceiling check, unlike
  `integrate_forward`'s continuous per-step check), and two additional
  mechanism-targeted probes both still succeeded rather than failed —
  evidence for, not proof of, that ceiling check protecting the branch.
  Verdict: cannot be determined further without disproportionate effort;
  not removed (not proven dead, upstream retains the identical branch
  with its own distinct diagnostic); UNFIXED. See "Uncovered, still no
  verdict" above.
- 1 finding produced a new `doc/upstream-bugs.md` entry
  (`totg-timing-zero-velocity-division`, `reproduced-grandfathered`) for
  the NaN/`+inf`-producing division itself, rather than a code fix.
- 17 sites are **not-this-family** (clause 1 — 16 `Display`/`Debug`
  success-path text checks from the original sweep, plus this round's
  `tests/ruckig_smoothing.rs:203` fixture-precondition check).
- 15 sites are **single-branch** (all found this round): each callee has
  exactly one `Error`/`None` construction site reachable from the
  asserting test, confirmed by reading the full function body, not by
  bite — there is no sibling branch to isolate from.
- 1 site is **structurally-redundant-operand**
  (`tests/robot_trajectory.rs:329`, unchanged from the original sweep,
  not re-investigated this round).
- The remaining 24 sites are **discriminating**: 23 from the original
  sweep (each confirmed by a live isolating-mutation bite, not by
  reading — `totg_compute_time_stamps`'s guard, `a_zero_custom_limit`'s
  guard (before the fix), `validate_group`, and `smoothness`'s guard were
  re-verified by fresh bite specifically in response to the
  funnel-shape/inverse-trap correction mid-round) plus this round's new
  `resample_dt_over_a_nan_duration_is_rejected` (`:1561`).
