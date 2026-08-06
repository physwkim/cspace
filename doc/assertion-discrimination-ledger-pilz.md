# Assertion-discrimination ledger — `moveit-planners-pilz`

Produced by p1-robotmodel, round 9, at the orchestrator's request: this is
the one crate in the 289-site census with no ledger, and two of its sites
were flagged as candidate clause-3 exclusions under
`doc/assertion-discrimination-census.md` §9. §1's mechanical scan finds 32
`matches!`/bare `.is_err()`/`.is_none()` sites in this crate; this table
classifies all 32.

Row set and most evidence come from p1-joints'
`01KZ7P8XWF3X0M69Q0NJJWMMT9-5.md` round report (absolute path:
`/home/stevek/work/moveit-rs/.caucus/sessions/01KZ7P482EAEQ840H8F9NKVWVW/rounds/`),
re-verified against the current tree — line numbers below are current-tree
line numbers and drift from that report's citations by a handful of lines
in several files (noted per row) due to unrelated intervening edits; the
sites themselves are unchanged. Three sites (the boundary-mismatch,
mismatched-sampling-time and non-stationary-waypoints guards of
`validate_request`) had no per-site evidence in that report beyond a
lumped count, so those three were freshly bitten this round instead of
cited.

Verdict legend: **discriminating** (proven to distinguish ≥2 sibling
outcomes, by bite, isolating fixture design, or structural single-code
argument), **single-branch** (exactly one possible construction site
reaches this assertion — nothing to discriminate), **joint-collapse** (in
family under census §9, but ≥2 real sibling guards fire together at this
assertion's fixture and it cannot say which — confirmed by bite, and
confirmed NOT a defect because no in-tree caller ever needs the
distinction and the collapse matches the ported function's own upstream
signature; distinct from `fixture-collapse-fixed`, which is an
*accidental* collapse this sweep fixes), **not-this-family** (excluded by
one of census §9's three clauses — cited per row).

Evidence legend: **commit** (a specific SHA whose message addresses this
exact site), **round-report** (p1-joints' report or an in-source comment,
re-read this round and independently verified against the current tree),
**bite** (a fresh reachability mutation run this round, reverted after
confirming).

## `trajectory_generator.rs` (14)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `trajectory_generator.rs:853` | `check_velocity_scaling`'s one `Error::Code(InvalidMotionPlan)` site (`is_scaling_factor_valid` false) | `scaling_factor_boundary_is_exclusive_below_and_inclusive_above` | single-branch | round-report (p1-joints, structural read this round: `check_velocity_scaling`'s body has exactly one `Error::` construction) |
| `trajectory_generator.rs:856` | same single site, upper-boundary value | same test fn | single-branch | same |
| `trajectory_generator.rs:858` | `check_acceleration_scaling`'s one `Error::Code(InvalidMotionPlan)` site | same test fn | single-branch | same structural read, `check_acceleration_scaling` |
| `trajectory_generator.rs:860` (`scaling_factor_boundary_is_exclusive_below_and_inclusive_above`) | same single site, upper-boundary value | same test fn | single-branch | same |
| `trajectory_generator.rs:865` | `check_velocity_scaling`'s single site (same as `:853`), zero input | `scaling_factor_rejects_zero_and_negative` | single-branch | same |
| `trajectory_generator.rs:866` (`scaling_factor_rejects_zero_and_negative`) | same single site, negative input | same test fn | single-branch | same |
| `trajectory_generator.rs:875` | `check_for_valid_group_name`'s one `Error::Code(InvalidGroupName)` site | `valid_group_name_accepted_unknown_group_name_rejected` | single-branch | round-report (p1-joints, structural read: 1 `Error::` construction in the function body) |
| `trajectory_generator.rs:904` | `check_start_state`'s position-limit-violation site (of 3 sites/2 codes: group-lookup→`InvalidGroupName`, position/velocity violation→`InvalidRobotState`) | `start_state_position_within_limit_accepted_beyond_limit_rejected` | discriminating | round-report (in-source doc comment on the test, verified: distinguishes the `InvalidGroupName` sibling; the position-vs-velocity collapse within `InvalidRobotState` is not a defect in its own right — it is still `discriminating` against the sibling that matters, since no caller-visible fact distinguishes position from velocity to begin with) |
| `trajectory_generator.rs:929` | same function, velocity-tolerance-violation site (mirror of `:904`) | `start_state_velocity_at_tolerance_accepted_beyond_it_rejected` | discriminating | same reasoning, mirror case |
| `trajectory_generator.rs:944` | same function, unknown-group site | `start_state_rejects_an_unknown_group` | discriminating | same doc comment: distinguishes both `InvalidRobotState` siblings by code |
| `trajectory_generator.rs:966` | `check_joint_goal`'s joint-outside-group site (of 3 sites/2 codes: group-lookup→`InvalidGroupName`, joint-outside-group and joint-beyond-limit→`InvalidGoalConstraints`) | `joint_goal_rejects_a_joint_outside_the_group` | discriminating | round-report (in-source doc comment, verified): distinguishes `InvalidGroupName`; the outside-group-vs-beyond-limit collapse within `InvalidGoalConstraints` is not a defect in its own right — no caller-visible fact distinguishes the two, so there is nothing this test could be blind to beyond the `InvalidGroupName` sibling it already discriminates |
| `trajectory_generator.rs:984` | same function, beyond-limit site (mirror of `:966`) | `joint_goal_within_limit_accepted_beyond_limit_rejected` | discriminating | same reasoning, mirror case |
| `trajectory_generator.rs:1001` | `check_cartesian_goal`'s empty-link-name site (of 2 sites, 2 distinct codes: empty link→`InvalidGoalConstraints`, no solver→`NoIkSolution`) | `cartesian_goal_rejects_an_empty_link_name` | discriminating | round-report (in-source doc comment, verified): the two codes are genuinely distinct outcomes, both individually tested |
| `trajectory_generator.rs:1023` | same function, no-solver-for-link site (mirror of `:1001`) | `cartesian_goal_rejects_a_non_tip_link` | discriminating | same reasoning, mirror case |

## `trajectory_generator_ptp.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `trajectory_generator_ptp.rs:648` | `plan_ptp`'s `InvalidGroupName` site (of 4 fallible sites: this one, 2× `PlanningFailed`, 1 sync-failed `Error::Construct`) | `plan_ptp_rejects_an_unknown_group` | discriminating | round-report (p1-joints): in-source doc comment gives the 4-site count (`rg -n 'Error::'` scoped to `plan_ptp`), checked structured code distinguishes from the other 3; cited `:620` then this round's orphan reconciliation found it drifted to `:648` (sibling test insertions earlier in the file) |
| `trajectory_generator_ptp.rs:734` | `MotionPlanResponse::failure`'s crate-wide-unique `trajectory: None` literal, reached by all of `generate`'s 5 failure short-circuits | `generate_rejects_an_invalid_group_before_planning` | single-branch | commit `02625e2` (p1-joints), re-verified: `rg -n 'trajectory: None'` crate-wide gives exactly 1 hit; which of the 5 failures fired is what the preceding `error_code` assertion already names — round-2 in-source audit comment at this site confirms the same; cited `:697` then `:705`, now `:734` — drifted twice by unrelated intervening edits |

## `path_circle.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `path_circle.rs:381` | `circle_from_center`'s one `Error::` site | `circle_from_center_radius_mismatch_is_rejected` | single-branch | commit `a1bbfe0` (p1-joints), re-verified via in-source round-2 audit comment (`rg -c 'Error::'` scoped to the function: 1); drift from cited `:376` |
| `path_circle.rs:449` | `circle_from_interim`'s one `Error::` site | `circle_from_interim_colinear_points_is_rejected` | single-branch | same commit/comment pattern, mirror function; drift from cited `:439` |

## `limits.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `limits.rs:571` | `JointLimitsContainer::limit`'s one `Error::` site | `limit_of_unknown_joint_is_an_error` | single-branch | commit `194b512` (p1-joints), re-verified via in-source round-2 audit comment (`rg -c 'Error::'` scoped to the function: 1); drift from cited `:567` |
| `limits.rs:581` | `common_limit_for`'s only fallible call, `self.limit(joint_name)?` — the same single `Error::` site `limit` has; its loop adds no second cause | `common_limit_for_unknown_joint_is_an_error` | single-branch | same commit; in-source comment states the loop calls the same single-site function, not a second cause; cited `:576` then `:584`, now `:581` — drifted twice by unrelated intervening edits, re-verified against current tree each time |

## `trajectory_blender_transition_window.rs` (8)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `trajectory_blender_transition_window.rs:774` | `validate_request`'s `blend_radius <= 0.0` guard (of 4 `InvalidMotionPlan` sites in this function) | `validate_request_rejects_blend_radius_at_or_below_zero` | discriminating | round-report (in-source comment on this test, verified): this round's fixture is deliberately chained so only this guard's precondition is violated; the comment records mutation testing that caught an earlier, non-chained fixture falsely passing via the boundary-mismatch guard instead |
| `trajectory_blender_transition_window.rs:779` (`validate_request_rejects_blend_radius_at_or_below_zero`) | same guard, second boundary value (`-0.01`) | same test fn | discriminating | same |
| `trajectory_blender_transition_window.rs:803` | same function, `is_robot_state_equal` boundary-mismatch guard | `validate_request_rejects_a_boundary_state_mismatch` | discriminating | bite (this round): neutralized this guard (`if false && !is_robot_state_equal(..)`) — `validate_request_rejects_a_boundary_state_mismatch` FAILED (no other guard fired for this fixture: `validate_request` returned `Ok`); reverted |
| `trajectory_blender_transition_window.rs:819` | same function, `determine_and_check_sampling_time` mismatch guard | `validate_request_rejects_a_mismatched_sampling_time` | discriminating | bite (this round): neutralized this guard (`.or(Some(0.1))` before the `.ok_or(..)`) — dedicated test FAILED, no other guard fired; reverted |
| `trajectory_blender_transition_window.rs:854` | same function, `is_robot_state_stationary` non-stationary-boundary guard (folds both trajectories' checks via `||` into one `Error::` site) | `validate_request_rejects_non_stationary_boundary_waypoints` | discriminating | bite (this round): neutralized this guard (`if false && (...)`) — dedicated test FAILED, no other guard fired; reverted. In-source comment on this test additionally records the fixture was built to hold the boundary-mismatch guard (`:803`'s subject) satisfied while only stationarity fails |
| `trajectory_blender_transition_window.rs:956` | `search_intersection_points`'s first-trajectory-search `ok_or` (of 2 sites sharing one code) | `search_intersection_points_rejects_when_first_trajectory_never_reaches_the_blend_radius` | discriminating | round-report (in-source comment above both tests, verified): each test keeps the OTHER trajectory a known crosser (case-A geometry) so only the trajectory under test can be the Err's cause — an isolating fixture split, commit `52d597b` per p1-joints |
| `trajectory_blender_transition_window.rs:972` | same function, second-trajectory-search `ok_or` (mirror of `:956`) | `search_intersection_points_rejects_when_second_trajectory_never_reaches_the_blend_radius` | discriminating | same isolating split, mirror case |
| `trajectory_blender_transition_window.rs:1211` | same function, both `ok_or` sites firing together at this pinned corner/radius fixture | `search_intersection_points_rejects_a_radius_that_exceeds_this_corners_reach` | joint-collapse | round-report (in-source comment directly above this test, verified, and p1-joints' own fresh bite this round: `.or(Some(0))` on the first search, reverted — confirmed the test stays green with either single guard disabled, only failing when both are neutralized). Confirmed NOT a defect: `search_intersection_points`'s own doc comment (`:381-385`) records both searches deliberately sharing one code, matching upstream's bool-only `searchIntersectionPoints`; `blend`, the only caller (`:234`), never distinguishes the two causes either — census §9a's `joint-collapse` verdict applies (all three clauses verified there), no fix needed |

## `pilz_trajectory_lin_parity.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `pilz_trajectory_lin_parity.rs:381` | `response.waypoints.is_none()` — `response` is the oracle's own recorded JSON fixture, deserialized, not a value the port produced | `lin_panda_arm_rejects_the_same_request_the_oracle_rejects` | not-this-family | census §9 clause 3 (subject): the in-source comment directly above this assertion already makes exactly this argument — "no guard or branch under test to discriminate; this only checks that the fixture itself is internally consistent" — deleting the port call from this test would not change this assertion's outcome; drift from p1-joints' cited `:366` |
| `pilz_trajectory_lin_parity.rs:447` | `response.trajectory.is_none()`, `generate_joint_trajectory`'s 2 `PlanningFailed` sites on this call path (`verify_sample_joint_limits` rejection vs. `push_way_point` failure) | `lin_panda_arm_rejects_the_same_request_the_oracle_rejects` | discriminating | commit `cedc581` (p1-joints), re-verified via in-source comment: isolating mutation neutralizing the `verify_sample_joint_limits` guard flipped this test from reject to accept, proving that site — not `push_way_point` — is this fixture's actual cause; drift from cited `:420` |

## `pilz_trajectory_circ_parity.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `pilz_trajectory_circ_parity.rs:389` | `response.waypoints.is_none()` — same shape as lin_parity's `:381`, oracle fixture precondition | `circ_panda_arm_rejects_the_same_request_the_oracle_rejects` | not-this-family | census §9 clause 3 (subject): identical in-source comment argument as lin_parity's `:381`; drift from cited `:373` |
| `pilz_trajectory_circ_parity.rs:444` | `response.trajectory.is_none()`, `TrajectoryGeneratorCirc::plan`/`build_path`'s 4 `InvalidMotionPlan` sites on this path | `circ_panda_arm_rejects_the_same_request_the_oracle_rejects` | discriminating | commit `6bf9388` (p1-joints), re-verified via in-source comment: `eprintln!` traced all 4 sites, only `PathCircle::new`'s no-determinable-plane check fired for this fixture; drift from cited `:415` |

## Summary

32 sites total. **30 in-family, 2 not-this-family** (both excluded by
census §9 clause 3 — the oracle-fixture-precondition shape, identical to
the `std::fs::read` example §9 itself resolves). Of the 30 in-family: 17
discriminating, 12 single-branch, 0 fixture-collapse-fixed, 1
joint-collapse (`:1211` — a genuine, confirmed-not-a-defect member, kept
out of the discriminating/single-branch split since it is neither).

Verdict tally check: 14 (`trajectory_generator.rs`) + 2 (`_ptp.rs`) + 2
(`path_circle.rs`) + 2 (`limits.rs`) + 8 (`trajectory_blender_transition_
window.rs`) + 2 (`lin_parity.rs`) + 2 (`circ_parity.rs`) = 32. Matches
p1-joints' per-file counts exactly; no reconciliation disagreement.

No source-changing commits — every mutation was reverted after
confirming.

## Commands run

- `cargo nextest run -p moveit-planners-pilz` (baseline, 145/145 pass;
  re-run after every revert, 145/145 pass each time)
- Bites (each: mutate, targeted `-E 'test(<fn>)'` run, revert,
  `git diff --stat` confirmed empty before continuing):
  - `validate_request_rejects_a_boundary_state_mismatch` — FAILED under
    mutation, confirming `:803`/`:352`
  - `validate_request_rejects_a_mismatched_sampling_time` — FAILED under
    mutation, confirming `:819`/`:357`
  - `validate_request_rejects_non_stationary_boundary_waypoints` — FAILED
    under mutation, confirming `:854`/`:368`
- `cargo fmt --all -- --check` — clean, before and after all bites

## Gate scope

`-p moveit-planners-pilz` for the bites above (each mutation gated by its
own targeted nextest run plus a full-crate re-run after revert).
`cargo fmt --all -- --check` for this document. No source diff to commit
— all mutations reverted. Full-workspace `--workspace` variants owed
before `git push` per the standing rule; not run this round.

## UNFIXED

None. `:1211`'s joint-collapse is confirmed not a defect (see its row);
nothing else in this crate's 32 sites is a genuinely blind assertion.

# Beyond the `matches!`/`.is_err()`/`.is_none()` grammar (12, then 13, now 24)

p1-joints, round following the one above. `tools/ci/count-coarse-assertions.py`
(the orchestrator's, committed `6a14a89` after `ledger_scan.py` — cited
nine times across ledgers including this one's original 32-site header —
was found to exist in no commit, worktree or checkout; see census §9c)
found 44 candidate sites in this crate at that round, of which 32 are the
`matches!`/`.is_err()`/`.is_none()` rows already classified above. The
remaining 12 were outside every prior instrument's grammar and had no
ledger row until then: 5 `contains_msg`, 4 `eq_none`, 2 `is_empty`, 1
`contains_member`. Both counts (44 total, 12 outside the old grammar,
and the 5/4/2/1 breakdown) were re-derived independently that round and
matched exactly.

**This round's orphan reconciliation** (p3-acm's closing audit, re-derived
independently per instruction rather than taken on the audit's word):
current scan is **45**, one more than the 44 above — this crate's own
`compute_link_fk` fix (commit `470362b`, already recorded in the `:1129`
row's evidence) added a new sibling test,
`compute_link_fk_rejects_the_bare_model_frame_when_it_is_not_a_link_name`,
whose own `eq_none` assertion (`:1168`) was never given its own row. Of
the 44 pre-existing rows, 10 had drifted (unrelated intervening edits
across several rounds shifted their citations, same test/same assert,
confirmed by test-function-name matching, not by line number) —
`limits.rs`'s 1, `trajectory_generator_ptp.rs`'s 5,
`trajectory_functions.rs`'s 4 — all corrected in place above and in the
32-site table above it. Zero raw genuine gaps beyond the one new test;
this crate's orphan count is **1**, not the brief's 12 (the brief's 12
counted every drifted citation as an orphan, the false-orphan trap named
in this round's instructions).

| file:line | kind | anchor | test fn | verdict | evidence |
|---|---|---|---|---|---|
| `path_circle.rs:558` | contains_msg | `PathCircle::new`'s zero-radius `Error::construct` (`"too small"`), sibling to the colinear guard below | `zero_radius_is_rejected` | discriminating | bite this round: `if false && radius < eps` falls through to the colinear guard (same fixture's `x_axis` zeroes, so `z_norm` also reads `< eps`); assertion correctly FAILS with the colinear message. Reverted |
| `path_circle.rs:590` | contains_msg | `PathCircle::new`'s colinear-plane `Error::construct` (`"colinear"`), sibling to the zero-radius guard above | `half_circle_from_center_has_no_determinable_plane` | discriminating | bite this round (mirror): `if false && z_norm < eps` lets construction succeed (`radius = 1.0` never triggers the other guard on this fixture) — assertion correctly FAILS, `unwrap_err()` panics on an `Ok(PathCircle { .. })`. Reverted |
| `trajectory_generator_ptp.rs:458` | contains_msg | `TrajectoryGeneratorPtp::new`'s joint-limit-not-set guard, one of 6 collapsed `Error::construct` sites | `constructor_rejects_missing_joint_limits` | discriminating | bite this round: `if false && !has_joint_limits` falls through to `common_limit_for` failing on an empty container — assertion correctly FAILS on "failed to compute common limit". Reverted. Cited `:449`, drifted to `:458` |
| `trajectory_generator_ptp.rs:487` | contains_msg | same function, `joint_model_group`'s `map_err`-wrapped invalid-group site | `constructor_rejects_unknown_group` | discriminating | not locally bite-able (sole gate on an external, borrowed-return `Result` in `moveit-model`, out of this crate's fence — no fallthrough value this crate can fabricate); discrimination instead checked by message uniqueness: "invalid group" appears in none of the other five literal message templates in this function. Cited `:466`, drifted to `:487` |
| `trajectory_generator_ptp.rs:524` | contains_msg | same function, acceleration-limit-not-set guard | `constructor_rejects_a_group_missing_an_acceleration_limit` | discriminating | bite this round: `if false && !has_acceleration_limits` falls through to the deceleration guard — assertion correctly FAILS on "deceleration limit not set for group panda_arm". Reverted. Cited `:495`, drifted to `:524` |
| `trajectory_functions.rs:974` | eq_none | `determine_and_check_sampling_time`'s folded `n1 < 2 && n2 < 2` guard (`doc/folded-operand-guards.md` names this site) | `determine_and_check_sampling_time_needs_at_least_two_intervals_on_one_side` | discriminating, but **blind operand pair — fixed** | bite both directions (`if n1 < 2 && true`, `if true && n2 < 2`) on this fixture (n1=n2=1): both left this test green, since either operand alone already makes the guard fire on this fixture. Fixed with two new isolating tests (n1=2/n2=1 and n1=1/n2=2); re-bit both mutations against all 6 tests — each now flips exactly the matching new test to a `None`-unwrap panic, sibling stays green. Commit `a53982f`. Cited `:948`, drifted to `:974` |
| `trajectory_functions.rs:1027` | eq_none | same function, first-trajectory interior-interval mismatch check | `determine_and_check_sampling_time_rejects_a_mismatched_interior_interval` | discriminating | bite this round: `if false && i < n1 && (...).abs() > epsilon` on the first-trajectory clause — assertion correctly FAILS, flipping from `None` to `Some(0.1)`; other 3 tests stay green. Reverted. Cited `:968`, drifted to `:1007` |
| `trajectory_functions.rs:1129` | eq_none | `compute_link_fk`'s `knows_frame_transform` guard, collapsing with the function's own fallthrough failure | `compute_link_fk_resolves_a_known_link_and_rejects_an_unknown_one` | discriminating, but **blind — fixed** | bite this round: `if false && !knows_frame_transform(..)` left this test green — the stock panda fixture's model frame is itself a real link, so `knows_frame_transform` and `frame_transform` never disagree on it (`moveit-state/src/state.rs:825`'s documented asymmetry needs a model frame that is *not* a link name). Fixed with a new test building a synthetic floating-virtual-joint SRDF (`model_frame() == "world"`, not a link) that forces the asymmetry; bite against it confirms discrimination. Commit `470362b`. Cited `:1070`, drifted to `:1129` |
| `trajectory_functions.rs:1168` | eq_none | `compute_link_fk`'s success-path return on the synthetic non-link-model-frame fixture — the new isolating test the `:1129` fix added | `compute_link_fk_rejects_the_bare_model_frame_when_it_is_not_a_link_name` | discriminating | this round's orphan reconciliation: this row never had its own citation — `:1129`'s row already carries its bite evidence (commit `470362b`) but only for the pre-fix test; this is that fix's own new sibling test, previously uncited |
| `trajectory_functions.rs:1244` | eq_none | `compute_pose_ik`'s tip-frame-mismatch guard, one of 5 collapsed `None`-producing paths | `compute_pose_ik_rejects_tip_frame_mismatch` | discriminating | bite this round: `if false && solver.tip_frame() != link_name` flips this test to an actual IK solution (`Some({"panda_joint7": ..})`) while `compute_pose_ik_round_trips_a_reachable_pose` stays green. Reverted. Commit `ab8439e` (doc only). Cited `:1137`, drifted to `:1224` |
| `path_line.rs:430` | contains_member | `assert!((0.0..=PI).contains(&angle))` on `get_rot_angle`'s return — a plain function, not `Result`/`Option` | `assert_get_rot_angle_round_trips` | not-this-family | census §9 clause 1 (mechanism): `angle` is a computed numeric value on the success path, not a coarse-fail signal; there is no guard to discriminate |
| `trajectory_blender_transition_window.rs:1271` | is_empty | `blend_sample_num`'s loop bound, arithmetic from indices with no branch producing "empty" as a signal | (blend-trajectory-cartesian test) | not-this-family | census §9 clause 1 (mechanism): success-path sanity check on a normally-computed range, not an inability signal |
| `path_rounded_composite.rs:417` | contains_msg | `PathRoundedComposite::new`'s sole rejection, non-positive `eqradius` | `new_rejects_a_non_positive_eqradius` | discriminating | bite this round: `if false && eqradius <= 0.0` lets construction succeed and `unwrap_err()` panics on an `Ok`; the other five `path_rounded_composite` rejection tests stay green. Reverted — citation re-derived this round to the `assert!` line; was line 412 |
| `path_rounded_composite.rs:439` | contains_msg | `add_corner`'s zero-length-incoming-leg rejection, one of 5 collapsed `Error::construct` sites in that function | `add_rejects_a_zero_length_incoming_segment` | discriminating | bite this round: `if false && incoming_len < ADD_EPSILON` flips exactly this test and no sibling — the needle `"arriving at this vertex coincides"` is emitted by no other guard. Reverted — citation re-derived this round to the `assert!` line; was line 434 |
| `path_rounded_composite.rs:457` | contains_msg | same function, zero-length-outgoing-leg rejection | `add_rejects_a_zero_length_outgoing_segment` | discriminating | bite this round (mirror of `:434`): flips exactly this test; needle `"leaving this vertex coincides"` is unique. Reverted — citation re-derived this round to the `assert!` line; was line 452 |
| `path_rounded_composite.rs:474` | contains_msg | same function, doubling-back rejection (interior angle within `ADD_EPSILON` of zero) | `add_rejects_a_reversing_corner` | discriminating | bite this round: `if false && theta < ADD_EPSILON` flips exactly this test — the fixture's reversed corner then reaches the tangent-length branch rather than a sibling guard. Reverted — citation re-derived this round to the `assert!` line; was line 469 |
| `path_rounded_composite.rs:490` | contains_msg | same function, radius-overruns-incoming-leg rejection | `add_rejects_a_radius_too_large_for_the_incoming_segment` | discriminating | bite this round: `if false && tangent_len >= incoming_len` flips exactly this test; the fixture's 10-long outgoing leg keeps the `:499` sibling from firing on it. Reverted — citation re-derived this round to the `assert!` line; was line 485 |
| `path_rounded_composite.rs:504` | contains_msg | same function, radius-overruns-outgoing-leg rejection | `add_rejects_a_radius_too_large_for_the_outgoing_segment` | discriminating | bite this round (mirror of `:485`, legs swapped): flips exactly this test. Reverted. All six bites run as one sweep; each produced a one-to-one guard→test failure with no sibling collateral — citation re-derived this round to the `assert!` line; was line 499 |
| `path_polyline_generator.rs:348` | contains_msg | `check_consecutive_colinear_waypoints`'s one `Error::construct` site, reached through `compute_blend_radius` | `compute_blend_radius_rejects_colinear_waypoints` | single-branch | structural read: the function has exactly one `Error::` construction (`rg -c 'Error::construct' ` over `check_consecutive_colinear_waypoints` = 1), so the message needle cannot be confused with a sibling's |
| `path_polyline_generator.rs:366` | contains_msg | same single site, reached on a corner whose legs are below `MIN_SEGMENT_LENGTH` — pins that the colinearity check runs *before* the short-leg `continue` | `compute_blend_radius_rejects_a_colinear_corner_whose_legs_are_too_short` | discriminating | bite this round: moving `check_consecutive_colinear_waypoints` after the `dist1/dist2 < MIN_SEGMENT_LENGTH` `continue` flips exactly this test (the corner is then skipped and `compute_blend_radius` returns `Ok`), 11 sibling tests stay green. Reverted |
| `path_polyline_generator.rs:418` | contains_msg | same single site, reached through `polyline_from_waypoints` rather than `compute_blend_radius` directly | `polyline_from_waypoints_rejects_a_colinear_run` | single-branch | same structural read as `:348`; this row's value is that the outer entry point propagates the rejection rather than swallowing it |
| `pilz_trajectory_polyline.rs:467` | is_none | `response.trajectory`, paired with the `assert_eq!(error_code, InvalidMotionPlan)` on the line above | `polyline_rejects_a_request_with_fewer_than_two_waypoints` | not-this-family | census §9 clause 1 (mechanism): the branch is already named by the `error_code` equality one line up; this is the companion check that a rejected response carries no trajectory, not the fail signal itself |
| `pilz_trajectory_polyline.rs:522` | is_none | same shape, on the variant-swap rejection | `polyline_plans_a_polyline_constraint_and_rejects_the_same_request_carrying_a_circ_one` | not-this-family | census §9 clause 1 (mechanism): same as `:467`. That test's own discrimination limit — `InvalidMotionPlan` cannot distinguish "constraint is CIRC" from "no POLYLINE waypoints" — is stated in its doc comment, and it pins the variant swap by asserting the un-swapped request reaches `Success` first |
| `trajectory_blender_transition_window.rs:1357` | is_empty | `response.blend_trajectory`, already known `Ok` via `.expect(..)` one line above | (blend test) | not-this-family | census §9 clause 1 (mechanism): same shape as `:1271` — a size check on a success-path collection, not a fail/absent signal |
| `pilz_trajectory_polyline_parity.rs:388` | is_none | `response.waypoints` — `response` is the oracle's own recorded JSON fixture, deserialized, not a value the port produced | `polyline_panda_arm_rejects_the_same_request_the_oracle_rejects` | not-this-family | census §9 clause 3 (subject): identical shape and identical argument to `pilz_trajectory_lin_parity.rs:381` and `pilz_trajectory_circ_parity.rs:389` — deleting the port call from this test would not change this assertion's outcome |
| `pilz_trajectory_polyline_parity.rs:413` | is_none | `response.trajectory`, from `TrajectoryGeneratorPolyline::cmd_specific_request_validation`'s waypoint-count guard — one of 3 `InvalidMotionPlan` sites reachable on this call path (that guard, and `polyline_path_constraint`'s absent- and wrong-variant returns) | `polyline_panda_arm_rejects_the_same_request_the_oracle_rejects` | discriminating | bite this round: relaxed the guard to `waypoints.len() < 1` — this assertion FAILED (the one-waypoint fixture then planned) together with its property-test sibling `polyline_rejects_a_request_with_fewer_than_two_waypoints`, and no other test in the crate moved (176/178). Reverted. The other two sites are additionally excluded within the test itself: it restores the dropped waypoint as its only edit and asserts `Success`, so neither the absent- nor the wrong-variant return can be this fixture's cause |
| `plan_components_builder.rs:467` | is_empty | `PlanComponentsBuilder::build`'s `traj_tail == None` path — the only path a builder that has never been appended to can take | `build_of_an_untouched_builder_is_empty` | not-this-family | census §9 clause 1 (mechanism): `build` returns `Ok` unconditionally when there is no tail, so there is no fail/absent signal and no sibling guard for the emptiness to be blind to. Same shape as `trajectory_blender_transition_window.rs:1357` — a size check on a success-path collection. The branch it does pin (`build` must not fabricate a container element) is bitten by the sibling `the_first_append_starts_one_container_element_that_build_flushes_the_tail_into`, whose `assert_eq!` is not a coarse-fail site |
| `command_list_manager.rs:812` | is_err | `solver_tip_frame` on the `hand` group — a fixture precondition, not the call under test (`extract_blend_radii`) | `a_radius_in_a_group_without_a_solver_is_zeroed` | not-this-family | census §9 clause 3 (subject): deleting `extract_blend_radii` from this test would not change this assertion's outcome. It exists so the test fails loudly, rather than passing vacuously, if `hand` ever gains a solver and the no-solver rule stops being the one that fires |
| `command_list_manager.rs:981` | is_empty | `solve`'s `items.is_empty()` early return | `an_empty_list_yields_an_empty_result_without_planning` | not-this-family | census §9 clause 1 (mechanism): the branch is named by the sibling `assert_eq!(*calls.borrow(), 0)` one line below, which is what distinguishes "returned before planning" from "planned nothing and produced nothing" — an empty result alone cannot tell those apart, since `PlanComponentsBuilder::build` on an untouched builder is also empty |

## Summary (12, then 13, then 26, then 27, now 29)

### The `CommandListManager` round's 2 additions

`command_list_manager.rs` arrived with the `CommandListManager` port;
`verify-orphan-enumeration.sh` reported its `is_err` fixture precondition and
its `is_empty` early-return check as orphans. Both are not-this-family, under
different clauses — one is not about the call under test at all, the other has
a sibling assertion that names the branch.

The port's own guards are not in this table because none of them is a coarse
fail/absent site: every one is checked by matching the returned
`SequenceError` variant *and* its fields. All eleven were bitten this round
(negative-radius, last-radius-zero, the per-group start-state rule, the
group-change and no-solver radius rules, the overlap group and zero-sum
guards, the start-state chaining, and the `Success`-without-trajectory
rejection), each flipping exactly its own test. Two more mutations bit
nothing and were treated as findings against the *code*, not the tests:
`is_invalid_blend_radii`'s zero-radius short-circuit decides nothing this port
can observe once upstream's warning is dropped (documented in place), and
`checkForOverlappingRadii`'s `size() < 3` guard existed only to keep
`size() - 2` from underflowing, so it was replaced by a saturating
subtraction — which the surviving test now bites at the pair-count boundary.

Table total re-derived after the additions, by parsing the table rather than
adding two to the previous figure: **29 rows — 14 `contains_msg`,
5 `eq_none`, 4 `is_none`, 4 `is_empty`, 1 `contains_member`, 1 `is_err`**; by
verdict, **16 discriminating, 2 discriminating but blind and since fixed, 9
not-this-family, 2 single-branch**.

### The `PlanComponentsBuilder` round's 1 addition

`plan_components_builder.rs` arrived with the `PlanComponentsBuilder` port
and had no ledger row; `verify-orphan-enumeration.sh` reported its single
`is_empty` site as an orphan. It is the success-path-collection shape
already resolved once in this file (`trajectory_blender_transition_window.rs:1357`).

The same run flagged 5 unresolved `trajectory_functions.rs` citations: this
round moved `solver_tip_frame` out of `trajectory_generator_lin.rs` and into
`trajectory_functions.rs` just above its test module, shifting all 5 by
exactly +20. Each was re-located inside the test function its row already
names, the same way the `trajectory_generator.rs` drift below was.

Table total re-derived after the addition, by parsing the table rather than
adding one to the previous figure: **27 rows — 14 `contains_msg`,
5 `eq_none`, 4 `is_none`, 3 `is_empty`, 1 `contains_member`**; by verdict,
**16 discriminating, 2 discriminating but blind and since fixed, 7
not-this-family, 2 single-branch**.

### The `smoothness_level` doc block's 16-line drift

Commit `1891736` added 16 lines of doc comment to `PolylinePathConstraint::
smoothness_level`, at `trajectory_generator.rs:231` — above the test module,
so all 14 of this file's citations moved by exactly +16 and
`verify-orphan-enumeration.sh` went from clean to 10 orphans / 9 unresolved
citations. Nothing about the assertions themselves changed; the gate was
not re-run after that commit, which is the only reason the drift shipped.

All 14 were re-derived the same way the previous drift was — by locating
the assertion inside the test function each row already names, not by
adding 16 to the old number and trusting it. The check that the shift was
uniform is that all 14 re-derived lines land inside the named function:
four in `scaling_factor_boundary_is_exclusive_below_and_inclusive_above`,
two in `scaling_factor_rejects_zero_and_negative`, and one each in the
remaining eight. A doc-only edit above a test module is the cheapest way to
break every citation in a file, and the ledger has now taken it twice.

### The POLYLINE-oracle round's 2 additions

`pilz_trajectory_polyline_parity.rs` arrived with the POLYLINE oracle op
(`tools/moveit-oracle/src/pilz_polyline_factory.cpp` and the
`panda_polyline_*` fixtures) and had no ledger row;
`verify-orphan-enumeration.sh` reported both of its `is_none` sites as
orphans. One is the oracle-fixture-precondition shape already resolved
twice in this file (LIN's `:381`, CIRC's `:389`); the other is bitten
above.

Table total re-derived after the additions, by parsing the table rather
than adding two to the previous figure: **26 rows — 14 `contains_msg`,
5 `eq_none`, 4 `is_none`, 2 `is_empty`, 1 `contains_member`**; by verdict,
**16 discriminating, 2 discriminating but blind and since fixed, 6
not-this-family, 2 single-branch**.

### The POLYLINE round's 11 additions

`path_rounded_composite`, `path_polyline_generator` and
`pilz_trajectory_polyline` arrived with the `POLYLINE` port and had no
ledger row; `verify-orphan-enumeration.sh` reported them as 11 orphans
against a baseline (`f5992d6`) that was clean, so all 11 are this round's.
Counts re-derived from the table above rather than carried forward, *as
the table stood at the end of that round*: **24 rows — 14 `contains_msg`,
5 `eq_none`, 2 `is_none`, 2 `is_empty`, 1 `contains_member`**; by verdict,
**15 discriminating, 2 discriminating but blind and since fixed, 5
not-this-family, 2 single-branch**. The current total is two rows higher —
see "The POLYLINE-oracle round's 2 additions" above.

Of the 11 new rows: 7 discriminating by live bite (the six
`path_rounded_composite` guards, each neutralized in turn in one sweep and
each flipping exactly its own test with no sibling collateral; plus
`path_polyline_generator.rs:366`'s check-order claim, bitten by moving the
colinearity check after the short-leg `continue`), 2 single-branch by
structural read (`:348`/`:418` reach the one `Error::construct` in
`check_consecutive_colinear_waypoints` through two different entry
points), and 2 not-this-family (`pilz_trajectory_polyline.rs`'s
`trajectory.is_none()` companions to an `error_code` equality that already
names the branch).

The same run found 10 ledger citations unresolved: the `PathConstraints`
enum shifted `trajectory_generator.rs` by 54–111 lines and `length_to_s`
shifted `path_line.rs`, and the reconciler's window then bound several
rows onto *each other's* assertions — `:805`'s target moved to `:859`,
which was itself another row's old citation. All 14
`trajectory_generator.rs` citations were therefore re-derived by test
function and assertion text, not only the 10 flagged.

### The preceding round's 13

**10 in-family, 3 not-this-family** (all three excluded by clause 1,
mechanism — `path_line.rs:430`, `trajectory_blender_transition_window.rs:1271`
and `:1357`; none is a coarse-fail signal). Of the 10 in-family: **8
discriminating with no fix needed** (7 from the original round plus this
round's new `:1168` row, itself already covered by `:1129`'s bite
evidence), and **2 were blind and are now fixed**
(`trajectory_functions.rs:974`'s folded `n1`/`n2` guard, `:1129`'s
model-frame/link-model asymmetry). One site
(`trajectory_generator_ptp.rs:487`) is discriminating by message
uniqueness rather than by live bite, for a structural reason (external,
borrowed-return `Result` this crate cannot fabricate a fallthrough value
for without editing `moveit-model`, out of fence) — stated rather than
silently skipped.

That round's figures (44 total, 12 outside the `matches!`/`.is_err()`/
`.is_none()` grammar, 5 `contains_msg`/4 `eq_none`/2 `is_empty`/1
`contains_member`) were exactly correct for that round's commit; this
round's independent re-derivation against the current tree gives 45/13,
reconciled above — see "This round's orphan reconciliation".

## Commands run (12)

- `python3 tools/ci/count-coarse-assertions.py crates/moveit-planners-pilz`
  — 44 hits, re-tabulated by kind, matched the user's figures exactly
- Bites (mutate with `if false && ...`, targeted `nextest -E` run,
  revert, confirm): `zero_radius_is_rejected` /
  `half_circle_from_center_has_no_determinable_plane` (mirror pair),
  `constructor_rejects_missing_joint_limits`,
  `constructor_rejects_a_group_missing_an_acceleration_limit`,
  `determine_and_check_sampling_time_*` (both directions, pre- and
  post-fix), `compute_link_fk_*` (pre- and post-fix),
  `compute_pose_ik_rejects_tip_frame_mismatch`
- `cargo fmt --all`, `cargo clippy -p moveit-planners-pilz --all-targets
  -- -D warnings`, `cargo nextest run -p moveit-planners-pilz` — full
  crate gate after all fixes: 148/148 pass (145 baseline + 3 new tests)

## Gate scope (12)

`-p moveit-planners-pilz` for all bites and fixes above (fmt, clippy
`--all-targets -D warnings`, nextest). Full-workspace `--workspace`
variants owed before `git push` per the standing rule; not run this
round.

## UNFIXED (12, now 13)

None. Both blind sites found that round (`trajectory_functions.rs:974`,
`:1129`, current-tree line numbers) are fixed with new isolating tests,
not comments. This round added one row (`:1168`) for a previously-uncited
site and corrected 10 drifted citations; no new blind site and nothing
UNFIXED from this round's orphan reconciliation.
