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
| `trajectory_generator.rs:783` | `check_velocity_scaling`'s one `Error::Code(InvalidMotionPlan)` site (`is_scaling_factor_valid` false) | `scaling_factor_boundary_is_exclusive_below_and_inclusive_above` | single-branch | round-report (p1-joints, structural read this round: `check_velocity_scaling`'s body has exactly one `Error::` construction) |
| `trajectory_generator.rs:786` | same single site, upper-boundary value | same test fn | single-branch | same |
| `trajectory_generator.rs:788` | `check_acceleration_scaling`'s one `Error::Code(InvalidMotionPlan)` site | same test fn | single-branch | same structural read, `check_acceleration_scaling` |
| `trajectory_generator.rs:790` | same single site, upper-boundary value | same test fn | single-branch | same |
| `trajectory_generator.rs:795` | `check_velocity_scaling`'s single site (same as `:783`), zero input | `scaling_factor_rejects_zero_and_negative` | single-branch | same |
| `trajectory_generator.rs:796` | same single site, negative input | same test fn | single-branch | same |
| `trajectory_generator.rs:805` | `check_for_valid_group_name`'s one `Error::Code(InvalidGroupName)` site | `valid_group_name_accepted_unknown_group_name_rejected` | single-branch | round-report (p1-joints, structural read: 1 `Error::` construction in the function body) |
| `trajectory_generator.rs:834` | `check_start_state`'s position-limit-violation site (of 3 sites/2 codes: group-lookup→`InvalidGroupName`, position/velocity violation→`InvalidRobotState`) | `start_state_position_within_limit_accepted_beyond_limit_rejected` | discriminating | round-report (in-source doc comment on the test, verified: distinguishes the `InvalidGroupName` sibling; the position-vs-velocity collapse within `InvalidRobotState` is not a defect in its own right — it is still `discriminating` against the sibling that matters, since no caller-visible fact distinguishes position from velocity to begin with) |
| `trajectory_generator.rs:859` | same function, velocity-tolerance-violation site (mirror of `:834`) | `start_state_velocity_at_tolerance_accepted_beyond_it_rejected` | discriminating | same reasoning, mirror case |
| `trajectory_generator.rs:874` | same function, unknown-group site | `start_state_rejects_an_unknown_group` | discriminating | same doc comment: distinguishes both `InvalidRobotState` siblings by code |
| `trajectory_generator.rs:896` | `check_joint_goal`'s joint-outside-group site (of 3 sites/2 codes: group-lookup→`InvalidGroupName`, joint-outside-group and joint-beyond-limit→`InvalidGoalConstraints`) | `joint_goal_rejects_a_joint_outside_the_group` | discriminating | round-report (in-source doc comment, verified): distinguishes `InvalidGroupName`; the outside-group-vs-beyond-limit collapse within `InvalidGoalConstraints` is not a defect in its own right — no caller-visible fact distinguishes the two, so there is nothing this test could be blind to beyond the `InvalidGroupName` sibling it already discriminates |
| `trajectory_generator.rs:914` | same function, beyond-limit site (mirror of `:896`) | `joint_goal_within_limit_accepted_beyond_limit_rejected` | discriminating | same reasoning, mirror case |
| `trajectory_generator.rs:931` | `check_cartesian_goal`'s empty-link-name site (of 2 sites, 2 distinct codes: empty link→`InvalidGoalConstraints`, no solver→`NoIkSolution`) | `cartesian_goal_rejects_an_empty_link_name` | discriminating | round-report (in-source doc comment, verified): the two codes are genuinely distinct outcomes, both individually tested |
| `trajectory_generator.rs:953` | same function, no-solver-for-link site (mirror of `:931`) | `cartesian_goal_rejects_a_non_tip_link` | discriminating | same reasoning, mirror case |

## `trajectory_generator_ptp.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `trajectory_generator_ptp.rs:620` | `plan_ptp`'s `InvalidGroupName` site (of 4 fallible sites: this one, 2× `PlanningFailed`, 1 sync-failed `Error::Construct`) | `plan_ptp_rejects_an_unknown_group` | discriminating | round-report (p1-joints; drift from cited `:620`→ line unchanged): in-source doc comment gives the 4-site count (`rg -n 'Error::'` scoped to `plan_ptp`), checked structured code distinguishes from the other 3 |
| `trajectory_generator_ptp.rs:705` | `MotionPlanResponse::failure`'s crate-wide-unique `trajectory: None` literal, reached by all of `generate`'s 5 failure short-circuits | `generate_rejects_an_invalid_group_before_planning` | single-branch | commit `02625e2` (p1-joints), re-verified: `rg -n 'trajectory: None'` crate-wide gives exactly 1 hit; which of the 5 failures fired is what the preceding `error_code` assertion already names — round-2 in-source audit comment at this site confirms the same, drift from p1-joints' cited `:697` (that line is the comment's start, not the assert) |

## `path_circle.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `path_circle.rs:381` | `circle_from_center`'s one `Error::` site | `circle_from_center_radius_mismatch_is_rejected` | single-branch | commit `a1bbfe0` (p1-joints), re-verified via in-source round-2 audit comment (`rg -c 'Error::'` scoped to the function: 1); drift from cited `:376` |
| `path_circle.rs:449` | `circle_from_interim`'s one `Error::` site | `circle_from_interim_colinear_points_is_rejected` | single-branch | same commit/comment pattern, mirror function; drift from cited `:439` |

## `limits.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `limits.rs:571` | `JointLimitsContainer::limit`'s one `Error::` site | `limit_of_unknown_joint_is_an_error` | single-branch | commit `194b512` (p1-joints), re-verified via in-source round-2 audit comment (`rg -c 'Error::'` scoped to the function: 1); drift from cited `:567` |
| `limits.rs:584` | `common_limit_for`'s only fallible call, `self.limit(joint_name)?` — the same single `Error::` site `limit` has; its loop adds no second cause | `common_limit_for_unknown_joint_is_an_error` | single-branch | same commit; in-source comment states the loop calls the same single-site function, not a second cause; drift from cited `:576` |

## `trajectory_blender_transition_window.rs` (8)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `trajectory_blender_transition_window.rs:762` | `validate_request`'s `blend_radius <= 0.0` guard (of 4 `InvalidMotionPlan` sites in this function) | `validate_request_rejects_blend_radius_at_or_below_zero` | discriminating | round-report (in-source comment on this test, verified): this round's fixture is deliberately chained so only this guard's precondition is violated; the comment records mutation testing that caught an earlier, non-chained fixture falsely passing via the boundary-mismatch guard instead |
| `trajectory_blender_transition_window.rs:767` | same guard, second boundary value (`-0.01`) | same test fn | discriminating | same |
| `trajectory_blender_transition_window.rs:791` | same function, `is_robot_state_equal` boundary-mismatch guard | `validate_request_rejects_a_boundary_state_mismatch` | discriminating | bite (this round): neutralized this guard (`if false && !is_robot_state_equal(..)`) — `validate_request_rejects_a_boundary_state_mismatch` FAILED (no other guard fired for this fixture: `validate_request` returned `Ok`); reverted |
| `trajectory_blender_transition_window.rs:807` | same function, `determine_and_check_sampling_time` mismatch guard | `validate_request_rejects_a_mismatched_sampling_time` | discriminating | bite (this round): neutralized this guard (`.or(Some(0.1))` before the `.ok_or(..)`) — dedicated test FAILED, no other guard fired; reverted |
| `trajectory_blender_transition_window.rs:842` | same function, `is_robot_state_stationary` non-stationary-boundary guard (folds both trajectories' checks via `||` into one `Error::` site) | `validate_request_rejects_non_stationary_boundary_waypoints` | discriminating | bite (this round): neutralized this guard (`if false && (...)`) — dedicated test FAILED, no other guard fired; reverted. In-source comment on this test additionally records the fixture was built to hold the boundary-mismatch guard (`:791`'s subject) satisfied while only stationarity fails |
| `trajectory_blender_transition_window.rs:944` | `search_intersection_points`'s first-trajectory-search `ok_or` (of 2 sites sharing one code) | `search_intersection_points_rejects_when_first_trajectory_never_reaches_the_blend_radius` | discriminating | round-report (in-source comment above both tests, verified): each test keeps the OTHER trajectory a known crosser (case-A geometry) so only the trajectory under test can be the Err's cause — an isolating fixture split, commit `52d597b` per p1-joints |
| `trajectory_blender_transition_window.rs:960` | same function, second-trajectory-search `ok_or` (mirror of `:944`) | `search_intersection_points_rejects_when_second_trajectory_never_reaches_the_blend_radius` | discriminating | same isolating split, mirror case |
| `trajectory_blender_transition_window.rs:1199` | same function, both `ok_or` sites firing together at this pinned corner/radius fixture | `search_intersection_points_rejects_a_radius_that_exceeds_this_corners_reach` | joint-collapse | round-report (in-source comment directly above this test, verified, and p1-joints' own fresh bite this round: `.or(Some(0))` on the first search, reverted — confirmed the test stays green with either single guard disabled, only failing when both are neutralized). Confirmed NOT a defect: `search_intersection_points`'s own doc comment (`:381-385`) records both searches deliberately sharing one code, matching upstream's bool-only `searchIntersectionPoints`; `blend`, the only caller (`:234`), never distinguishes the two causes either — census §9a's `joint-collapse` verdict applies (all three clauses verified there), no fix needed |

## `pilz_trajectory_lin_parity.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `pilz_trajectory_lin_parity.rs:371` | `response.waypoints.is_none()` — `response` is the oracle's own recorded JSON fixture, deserialized, not a value the port produced | `lin_panda_arm_rejects_the_same_request_the_oracle_rejects` | not-this-family | census §9 clause 3 (subject): the in-source comment directly above this assertion already makes exactly this argument — "no guard or branch under test to discriminate; this only checks that the fixture itself is internally consistent" — deleting the port call from this test would not change this assertion's outcome; drift from p1-joints' cited `:366` |
| `pilz_trajectory_lin_parity.rs:437` | `response.trajectory.is_none()`, `generate_joint_trajectory`'s 2 `PlanningFailed` sites on this call path (`verify_sample_joint_limits` rejection vs. `push_way_point` failure) | `lin_panda_arm_rejects_the_same_request_the_oracle_rejects` | discriminating | commit `cedc581` (p1-joints), re-verified via in-source comment: isolating mutation neutralizing the `verify_sample_joint_limits` guard flipped this test from reject to accept, proving that site — not `push_way_point` — is this fixture's actual cause; drift from cited `:420` |

## `pilz_trajectory_circ_parity.rs` (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `pilz_trajectory_circ_parity.rs:378` | `response.waypoints.is_none()` — same shape as lin_parity's `:371`, oracle fixture precondition | `circ_panda_arm_rejects_the_same_request_the_oracle_rejects` | not-this-family | census §9 clause 3 (subject): identical in-source comment argument as lin_parity's `:371`; drift from cited `:373` |
| `pilz_trajectory_circ_parity.rs:433` | `response.trajectory.is_none()`, `TrajectoryGeneratorCirc::plan`/`build_path`'s 4 `InvalidMotionPlan` sites on this path | `circ_panda_arm_rejects_the_same_request_the_oracle_rejects` | discriminating | commit `6bf9388` (p1-joints), re-verified via in-source comment: `eprintln!` traced all 4 sites, only `PathCircle::new`'s no-determinable-plane check fired for this fixture; drift from cited `:415` |

## Summary

32 sites total. **30 in-family, 2 not-this-family** (both excluded by
census §9 clause 3 — the oracle-fixture-precondition shape, identical to
the `std::fs::read` example §9 itself resolves). Of the 30 in-family: 17
discriminating, 12 single-branch, 0 fixture-collapse-fixed, 1
joint-collapse (`:1199` — a genuine, confirmed-not-a-defect member, kept
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
    mutation, confirming `:791`/`:352`
  - `validate_request_rejects_a_mismatched_sampling_time` — FAILED under
    mutation, confirming `:807`/`:357`
  - `validate_request_rejects_non_stationary_boundary_waypoints` — FAILED
    under mutation, confirming `:842`/`:368`
- `cargo fmt --all -- --check` — clean, before and after all bites

## Gate scope

`-p moveit-planners-pilz` for the bites above (each mutation gated by its
own targeted nextest run plus a full-crate re-run after revert).
`cargo fmt --all -- --check` for this document. No source diff to commit
— all mutations reverted. Full-workspace `--workspace` variants owed
before `git push` per the standing rule; not run this round.

## UNFIXED

None. `:1199`'s joint-collapse is confirmed not a defect (see its row);
nothing else in this crate's 32 sites is a genuinely blind assertion.

# Beyond the `matches!`/`.is_err()`/`.is_none()` grammar (12)

p1-joints, round following the one above. `tools/ci/count-coarse-assertions.py`
(the orchestrator's, committed `6a14a89` after `ledger_scan.py` — cited
nine times across ledgers including this one's original 32-site header —
was found to exist in no commit, worktree or checkout; see census §9c)
finds 44 candidate sites in this crate, of which 32 are the
`matches!`/`.is_err()`/`.is_none()` rows already classified above. The
remaining 12 are outside every prior instrument's grammar and had no
ledger row until now: 5 `contains_msg`, 4 `eq_none`, 2 `is_empty`, 1
`contains_member`. Both counts (44 total, 12 outside the old grammar,
and the 5/4/2/1 breakdown) were re-derived independently this round and
matched exactly.

| file:line | kind | anchor | test fn | verdict | evidence |
|---|---|---|---|---|---|
| `path_circle.rs:558` | contains_msg | `PathCircle::new`'s zero-radius `Error::construct` (`"too small"`), sibling to the colinear guard below | `zero_radius_is_rejected` | discriminating | bite this round: `if false && radius < eps` falls through to the colinear guard (same fixture's `x_axis` zeroes, so `z_norm` also reads `< eps`); assertion correctly FAILS with the colinear message. Reverted |
| `path_circle.rs:590` | contains_msg | `PathCircle::new`'s colinear-plane `Error::construct` (`"colinear"`), sibling to the zero-radius guard above | `half_circle_from_center_has_no_determinable_plane` | discriminating | bite this round (mirror): `if false && z_norm < eps` lets construction succeed (`radius = 1.0` never triggers the other guard on this fixture) — assertion correctly FAILS, `unwrap_err()` panics on an `Ok(PathCircle { .. })`. Reverted |
| `trajectory_generator_ptp.rs:449` | contains_msg | `TrajectoryGeneratorPtp::new`'s joint-limit-not-set guard, one of 6 collapsed `Error::construct` sites | `constructor_rejects_missing_joint_limits` | discriminating | bite this round: `if false && !has_joint_limits` falls through to `common_limit_for` failing on an empty container — assertion correctly FAILS on "failed to compute common limit". Reverted |
| `trajectory_generator_ptp.rs:466` | contains_msg | same function, `joint_model_group`'s `map_err`-wrapped invalid-group site | `constructor_rejects_unknown_group` | discriminating | not locally bite-able (sole gate on an external, borrowed-return `Result` in `moveit-model`, out of this crate's fence — no fallthrough value this crate can fabricate); discrimination instead checked by message uniqueness: "invalid group" appears in none of the other five literal message templates in this function |
| `trajectory_generator_ptp.rs:495` | contains_msg | same function, acceleration-limit-not-set guard | `constructor_rejects_a_group_missing_an_acceleration_limit` | discriminating | bite this round: `if false && !has_acceleration_limits` falls through to the deceleration guard — assertion correctly FAILS on "deceleration limit not set for group panda_arm". Reverted |
| `trajectory_functions.rs:948` | eq_none | `determine_and_check_sampling_time`'s folded `n1 < 2 && n2 < 2` guard (`doc/folded-operand-guards.md` names this site) | `determine_and_check_sampling_time_needs_at_least_two_intervals_on_one_side` | discriminating, but **blind operand pair — fixed** | bite both directions (`if n1 < 2 && true`, `if true && n2 < 2`) on this fixture (n1=n2=1): both left this test green, since either operand alone already makes the guard fire on this fixture. Fixed with two new isolating tests (n1=2/n2=1 and n1=1/n2=2); re-bit both mutations against all 6 tests — each now flips exactly the matching new test to a `None`-unwrap panic, sibling stays green. Commit `a53982f` |
| `trajectory_functions.rs:968` | eq_none | same function, first-trajectory interior-interval mismatch check | `determine_and_check_sampling_time_rejects_a_mismatched_interior_interval` | discriminating | bite this round: `if false && i < n1 && (...).abs() > epsilon` on the first-trajectory clause — assertion correctly FAILS, flipping from `None` to `Some(0.1)`; other 3 tests stay green. Reverted |
| `trajectory_functions.rs:1070` | eq_none | `compute_link_fk`'s `knows_frame_transform` guard, collapsing with the function's own fallthrough failure | `compute_link_fk_resolves_a_known_link_and_rejects_an_unknown_one` | discriminating, but **blind — fixed** | bite this round: `if false && !knows_frame_transform(..)` left this test green — the stock panda fixture's model frame is itself a real link, so `knows_frame_transform` and `frame_transform` never disagree on it (`moveit-state/src/state.rs:825`'s documented asymmetry needs a model frame that is *not* a link name). Fixed with a new test building a synthetic floating-virtual-joint SRDF (`model_frame() == "world"`, not a link) that forces the asymmetry; bite against it confirms discrimination. Commit `470362b` |
| `trajectory_functions.rs:1137` | eq_none | `compute_pose_ik`'s tip-frame-mismatch guard, one of 5 collapsed `None`-producing paths | `compute_pose_ik_rejects_tip_frame_mismatch` | discriminating | bite this round: `if false && solver.tip_frame() != link_name` flips this test to an actual IK solution (`Some({"panda_joint7": ..})`) while `compute_pose_ik_round_trips_a_reachable_pose` stays green. Reverted. Commit `ab8439e` (doc only) |
| `path_line.rs:417` | contains_member | `assert!((0.0..=PI).contains(&angle))` on `get_rot_angle`'s return — a plain function, not `Result`/`Option` | `assert_get_rot_angle_round_trips` | not-this-family | census §9 clause 1 (mechanism): `angle` is a computed numeric value on the success path, not a coarse-fail signal; there is no guard to discriminate |
| `trajectory_blender_transition_window.rs:1259` | is_empty | `blend_sample_num`'s loop bound, arithmetic from indices with no branch producing "empty" as a signal | (blend-trajectory-cartesian test) | not-this-family | census §9 clause 1 (mechanism): success-path sanity check on a normally-computed range, not an inability signal |
| `trajectory_blender_transition_window.rs:1345` | is_empty | `response.blend_trajectory`, already known `Ok` via `.expect(..)` one line above | (blend test) | not-this-family | census §9 clause 1 (mechanism): same shape as `:1259` — a size check on a success-path collection, not a fail/absent signal |

## Summary (12)

**9 in-family, 3 not-this-family** (all three excluded by clause 1,
mechanism — `path_line.rs:417`, `trajectory_blender_transition_window.rs:1259`
and `:1345`; none is a coarse-fail signal). Of the 9 in-family: **7
discriminating with no fix needed**, and **2 were blind and are now
fixed** (`trajectory_functions.rs:948`'s folded `n1`/`n2` guard,
`:1070`'s model-frame/link-model asymmetry). One site
(`trajectory_generator_ptp.rs:466`) is discriminating by message
uniqueness rather than by live bite, for a structural reason (external,
borrowed-return `Result` this crate cannot fabricate a fallthrough value
for without editing `moveit-model`, out of fence) — stated rather than
silently skipped.

The user's figures (44 total, 12 outside the `matches!`/`.is_err()`/
`.is_none()` grammar, 5 `contains_msg`/4 `eq_none`/2 `is_empty`/1
`contains_member`) were exactly correct; no discrepancy.

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

## UNFIXED (12)

None. Both blind sites found this round (`trajectory_functions.rs:948`,
`:1070`) are fixed with new isolating tests, not comments.
