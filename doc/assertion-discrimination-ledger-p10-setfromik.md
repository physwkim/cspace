# Assertion-discrimination ledger — p10-setfromik (`moveit-kinematics::set_from_ik`)

The nine sites the sweep's scanner emits for the `setFromIK` port. All nine
are new test assertions in
`crates/moveit-kinematics/tests/set_from_ik.rs`, and all nine are of the
one shape the scanner scores here: `assert!(matches!(error, ...))` on an
error the port returned. That shape is exactly the one that can fail to
discriminate — `Error::Other` is one variant with nine construction sites
across this port, and `Error::UnknownName` is one variant with three
`kind` strings — so every row below is a claim about *which* site the test
can name, not merely that some error came back.

The port's other guards produce no scanner site (they are compared with
`assert_eq!` on positions, or with a tolerance) and so are outside this
sweep's corpus. They were bitten anyway; section 4 records those bites,
because a guard whose bite was never run is a guard whose test was never
checked, corpus or no corpus.

## Method

Every row's evidence is a mutation applied to this worktree, run with
`cargo nextest run -p <crate> --no-fail-fast`, and reverted, with the
revert confirmed by `git status --porcelain` naming no mutated file. No row
is justified by reading the code alone.

Baselines at the time of the bites: `moveit-kinematics` 76 tests / 76
passing, `moveit-model` 144 tests / 144 passing.

One correction is recorded in place rather than silently fixed. The first
harness scored only lines containing `FAIL`, and this repository's
`.config/nextest.toml` sets `slow-timeout = { period = "60s",
terminate-after = 5 }`, which reports a hung test as **TIMEOUT** — so bite
B21 came back "no test failed" when in fact three tests had hung for 300s
each. The harness now scores `FAIL`, `TIMEOUT`, `SIGSEGV`, `ABORT`, `LEAK`,
`SIGABRT` and `CANCEL`, and B21's real result is in section 4. A bite that
turns a guard's regression signal from a failure into a stall is still a
bite; a harness that cannot see the difference is not evidence.

## 1. `Error::Other` sites — five distinct messages, five distinct tests

Five of this port's nine `Error::other` construction sites are reachable
from a test. Each assertion matches on a substring that only its own site
emits, so the rows below claim discrimination rather than single-branch.

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-kinematics/tests/set_from_ik.rs:377` | `matches!(error, Error::Other(ref m) if m.contains("panda_link6"))` — the `let Some(tip_id) = matched else` arm of `resolve_ik_queries` (`crates/moveit-kinematics/src/set_from_ik.rs:365-372`) | `a_target_naming_a_frame_across_a_moving_joint_is_not_a_tip_match` | discriminating | Bite B3b: `... == rigid_parent_link(model, attached, tip)? \|\| true`, so every frame matches the first unclaimed tip. That one test FAILS (it gets `Ok` where it asserted `Err`); the other 75 stay green, including `:474`, the only other test that reaches this same `else`. The substring is the *cited frame name*, which the message interpolates from the target, so a different target reaching the same site prints a different name. |
| `crates/moveit-kinematics/tests/set_from_ik.rs:474` | `matches!(error, Error::Other(ref m) if m.contains("IK target 1"))` — the same `else`, reached because `claimed[tip_id]` skipped the only tip | `a_tip_a_target_already_claimed_cannot_be_matched_twice` | discriminating | Bite B4: delete `if claimed[tip_id] { continue; }` from the *matching* loop, leaving the fill loop's copy in place. That one test FAILS; 75 stay green, `:377` among them. The two rows are each other's isolating evidence: B3b fails `:377` and not `:474`, B4 fails `:474` and not `:377`, so neither is reading the other's branch. The index in `IK target 1` is the loop counter, so it also pins *which* target was refused. |
| `crates/moveit-kinematics/tests/set_from_ik.rs:726` | `matches!(error, Error::Other(ref m) if m.contains("set_from_ik_subgroups"))` — `set_from_ik`'s `let [target] = queries.as_slice() else` (`crates/moveit-kinematics/src/set_from_ik.rs:497-504`) | `a_solver_reporting_two_tips_is_refused_rather_than_solved_for_the_first` | discriminating | Bite B10: `let [target, ..] = queries.as_slice() else`, which accepts any non-empty query list and silently solves the first tip. That one test FAILS; 75 stay green — including `a_tip_no_target_named_is_filled_with_the_pose_it_currently_holds`, which drives the same two-tip solver through `resolve_ik_queries` directly and so must not move when this refusal does. |
| `crates/moveit-kinematics/tests/set_from_ik.rs:1225` | `matches!(error, Error::Other(ref m) if m.contains("at least one"))` — `set_from_ik_subgroups`' `solvers.is_empty()` guard | `no_subgroup_solvers_is_an_error_not_a_vacuous_success` | discriminating | Bite B15: `if false {`. That one test FAILS; 75 stay green, `:1251` among them. Necessary as its own guard rather than a consequence of the length check: with zero solvers *and* zero targets the length check passes, and the sweep loop then runs zero subgroups and returns `Ok(true)` — a success that solved nothing. B15's failure is exactly that `Ok(true)`. |
| `crates/moveit-kinematics/tests/set_from_ik.rs:1251` | `matches!(error, Error::Other(ref m) if m.contains("2 subgroup solvers for 1 targets"))` — the `solvers.len() != targets.len()` guard | `one_target_per_subgroup_solver_or_it_is_an_error` | discriminating | Bite B14: `if false {`. That one test FAILS; 75 stay green, `:1225` among them. The substring carries both counts, so it separates this site from `:1225`'s (which names neither) and would not survive the counts being transposed. |
| `crates/moveit-kinematics/tests/set_from_ik.rs:1280` | `matches!(error, Error::Other(ref m) if m.contains("not a subgroup"))` — the `group.is_subgroup(solver.group_name())` guard | `a_solver_for_a_group_that_is_not_a_subgroup_is_refused` | discriminating | Bite B16: `if false {`. That one test FAILS; 75 stay green. The fixture solver is `left_arm`'s, relabelled to claim the group `base`, so the solve itself would succeed if the guard let it through — the test is measuring the guard and not the solver. |

## 2. `Error::UnknownName` sites — the `kind` string is the discriminator

`Error::UnknownName` carries a `kind: &'static str`. Three of this port's
sites use three different kinds, and each assertion matches on its own,
which is what lets these three rows claim discrimination rather than
collapsing into "some name was unknown".

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-kinematics/tests/set_from_ik.rs:400` | `matches!(error, Error::UnknownName { kind: "IK frame", ref name } if name == "no_such_frame")` — `rigid_parent_link`'s attached-frame fallback (`crates/moveit-kinematics/src/set_from_ik.rs:268-270`) | `a_target_naming_nothing_in_the_model_is_unknown_name_not_no_match` | discriminating | Bite B22: change that one site's kind from `"IK frame"` to `"link"`, leaving the variant, the name and every other site untouched. That one test FAILS; 75 stay green, `:609` among them — so the assertion is reading the kind, not merely the variant, and the two `UnknownName` sites in this file are told apart by it. |
| `crates/moveit-kinematics/tests/set_from_ik.rs:609` | `matches!(error, Error::UnknownName { kind: "link", ref name } if name == "no_such_base")` — `to_solver_frame`'s `posed.global_link_transform(ik_frame)?` on a base frame that is neither the model frame nor a link | `a_solver_based_on_a_frame_the_model_does_not_have_is_unknown_name` | discriminating | Bite B23: `.unwrap_or_else(\|_\| Isometry3::identity())` on that transform lookup, so an unknown base silently becomes the model frame. That one test FAILS; 75 stay green, `:400` among them. |
| `crates/moveit-kinematics/tests/set_from_ik.rs:753` | `matches!(error, Error::UnknownName { kind: "group variable", ref name } if name == "panda_joint1")` — `check_solver_joints_are_group_variables` | `a_solver_whose_joints_are_not_the_groups_variables_is_unknown_name` | discriminating | Bite B9b: `&& false` on the membership test, so no solver joint is ever rejected. That one test FAILS; 75 stay green. The name asserted is the *first* offending joint, so the row also pins that the check reports at the first failure rather than after the loop. |

## 3. What the corpus cannot see

The scanner scores `matches!` and `eq_none` shapes. It does not score
`assert_eq!(state.positions(), entry.as_slice())`, which is how the
state-restoration invariant — the one place this port deliberately
diverges from upstream — is tested. Those assertions are therefore absent
from the orphan list and no row above accounts for them. They are bitten in
section 4 all the same.

## 4. Bites on guards outside the corpus

Same harness, same revert discipline. Rows are grouped where one mutation
fails several tests, because reporting a family bite as an exclusive one
overstates what it proved.

| bite | mutation | tests that failed | reading |
|---|---|---|---|
| B2b | in the rigid-connection carry, replace the tip's transform with the pose frame's own, making the product identity | `a_target_naming_a_welded_frame_is_carried_across_to_the_tip`, `a_welded_frames_target_is_solved_for_that_frame_not_for_the_tip`, `an_attached_frame_reaches_the_tip_through_the_link_it_hangs_from` | Family of three, correctly: every test whose target frame is not the tip itself goes through this one multiply. The `CARRY_TOL_RAD` bound (`1e-14`) is 13 orders of magnitude below the 0.785 rad the identity product leaves behind, so the failure is not a tolerance artefact. |
| B4 | delete `if claimed[tip_id] { continue; }` from the matching loop | `a_tip_a_target_already_claimed_cannot_be_matched_twice` | Exclusive. |
| B5 | `if true { continue; }` in the *fill* loop, so unnamed tips keep the identity they were initialised with | `a_tip_no_target_named_is_filled_with_the_pose_it_currently_holds`, `naming_no_target_at_all_fills_every_tip_and_asks_the_solver_to_stay` | Family of two — the only two tests that own a tip no target names. Paired with B4, which touches the matching loop's copy of the same `claimed` read and fails neither of these: the two loops are separately covered. |
| B6 | `if false && Transforms::same_frame(...)` in `to_solver_frame`, forcing a link lookup for a base frame that is the model frame | `a_solver_based_on_the_model_frame_needs_no_link_of_that_name` | Exclusive, and it is only exclusive because this fixture's model frame (`"world"`) is not a link — the test asserts that precondition itself, so it fails loudly rather than silently stopping discriminating if a future fixture adds a `world` link. |
| B7 | in `rigid_parent_link`, resolve an attached frame to `panda_link0` instead of to the link it hangs from | `an_attached_frame_reaches_the_tip_through_the_link_it_hangs_from` | Exclusive — but see B8. |
| B8 | in `frame_transform`, drop the `* body.link_pose_frame` factor | `an_attached_frame_reaches_the_tip_through_the_link_it_hangs_from` | Exclusive, and **the same single test as B7**. Two independent guards, one test: the attached-frames tier has one boundary case covering both its lookup and its transform. Recorded rather than presented as two clean bites; a third attached body whose transform is identity would separate them. |
| B9b | `&& false` on the solver-joint membership test | `a_solver_whose_joints_are_not_the_groups_variables_is_unknown_name` | Exclusive. |
| B10 | `let [target, ..]` instead of `let [target]` | `a_solver_reporting_two_tips_is_refused_rather_than_solved_for_the_first` | Exclusive. |
| B11b | delete `state.set_variable_positions(&entry_positions)` from `set_from_ik` | `a_rejecting_hook_reports_no_solution_and_rewinds_what_it_wrote`, `an_accepting_hooks_writes_outside_the_group_do_not_survive` | Family of two, and this is the port's central deviation from upstream: without the restore the state keeps whatever the validity hook last wrote. `an_unreachable_target_leaves_the_state_exactly_as_it_found_it` stays green under B11b, because with no hook nothing writes the state before the restore — so that test measures the no-hook path and these two measure the hook path. |
| B12 | in `apply_and_read_group`, return the solver's own solution instead of reading the group's variables back | `the_hook_receives_every_group_variable_including_the_mimic_no_solver_writes` | Exclusive. This is the bijection question: the fixture's solver reports one joint and the group has two variables, so a port that permutes the solution hands the hook one value and this port hands it two. |
| B13 | in `apply_and_read_group`, stop writing the candidate into the state | `the_hook_receives_every_group_variable_including_the_mimic_no_solver_writes`, `the_hook_sees_a_state_that_already_holds_the_candidate` | Family of two. The mimic test fails as well because the mimic's value only exists once the write propagates; there is no separating mutation, since the read-back has nothing to read without the write. |
| B14 | `if false` on the subgroup length check | `one_target_per_subgroup_solver_or_it_is_an_error` | Exclusive. |
| B15 | `if false` on the empty-solvers check | `no_subgroup_solvers_is_an_error_not_a_vacuous_success` | Exclusive. |
| B16 | `if false` on the subgroup-membership check | `a_solver_for_a_group_that_is_not_a_subgroup_is_refused` | Exclusive. |
| B17b | delete the rewind at the foot of `set_from_ik_subgroups`' attempt loop | `a_rejecting_group_hook_rewinds_every_sweep_it_refuses`, `a_subgroup_that_cannot_reach_rewinds_the_one_that_could` | Family of two — the two ways a sweep can fail (a subgroup that will not solve, and a group hook that will not accept), which is the whole boundary. |
| B18 | drop the re-application of the sweep's own positions after an accepting hook | `an_accepting_group_hooks_writes_do_not_survive_the_sweep` | Exclusive — **and this bite is why that test exists**. The first pass through this ledger ran B18 against the file as first written and nothing failed: every subgroup test either had no hook or had a rejecting one, so the accepting-hook-writes-elsewhere boundary was untested. The test was added, and B18 then failed it alone. |
| B19 | `let held_still = false`, so the walk stops at the first joint | `rigidly_connected_parent_walks_up_a_fixed_joint`, `rigidly_connected_parent_crosses_a_whole_run_of_fixed_joints`, `a_movable_joint_inside_the_group_stops_the_walk`, `a_movable_joint_outside_the_group_is_held_still_and_walked_past` | Family of four — every test whose answer is not the link it started from. `rigidly_connected_parent_stops_at_the_joint_that_can_move_the_link` and `..._of_the_root_link_is_the_root` stay green, correctly: their answer *is* the starting link. |
| B20b | keep the group membership test but force its result false, so an out-of-group joint is no longer held still | `a_movable_joint_outside_the_group_is_held_still_and_walked_past` | Exclusive. Its sibling `a_movable_joint_inside_the_group_stops_the_walk` stays green, so the pair separates the two sides of the membership test rather than both riding on the group argument being present. |
| B21 | `.or(Some(current))` on the parent-link lookup, removing the walk's `None` exit | TIMEOUT ×3: `rigidly_connected_parent_of_the_root_link_is_the_root`, `rigidly_connected_parent_crosses_a_whole_run_of_fixed_joints`, `a_movable_joint_outside_the_group_is_held_still_and_walked_past` | Family of three — every test whose walk reaches the root. Probed the fixture directly to confirm the mechanism rather than infer it: the root link `base` has `parent_link_index() == None` and a `Fixed` parent joint (the SRDF virtual joint), so with the `None` exit removed `held_still` stays true and `current` never advances. The signal is a hang, not a wrong answer, which is why this bite is only visible with `terminate-after` set and a harness that scores TIMEOUT. |
| B21b | return the *parent* of the link the walk stopped at | `rigidly_connected_parent_walks_up_a_fixed_joint`, `rigidly_connected_parent_stops_at_the_joint_that_can_move_the_link`, `a_movable_joint_inside_the_group_stops_the_walk` | Family of three, and it leaves `..._of_the_root_link_is_the_root` green — the root has no parent, so `unwrap_or(current)` returns the root either way. Taken with B21, the root test is covered only as part of a family: no mutation found here fails it alone while leaving its siblings green. |
| B22 | change `rigid_parent_link`'s unknown-frame `kind` from `"IK frame"` to `"link"` | `a_target_naming_nothing_in_the_model_is_unknown_name_not_no_match` | Exclusive. |
| B23 | swallow `to_solver_frame`'s base-frame lookup error into the identity transform | `a_solver_based_on_a_frame_the_model_does_not_have_is_unknown_name` | Exclusive. |

## 5. One guard no test isolates

| bite | mutation | tests that failed | reading |
|---|---|---|---|
| B1 | delete `resolve_ik_queries`' exact-name fast path, `if pose_frame == tip { matched = Some(tip_id); break; }` | none — all 76 pass | Not a coverage gap: the branch is behaviourally redundant. For a target naming the tip exactly, the rigid-connection branch below it compares `rigid_parent_link(pose_frame)` against `rigid_parent_link(tip)` for the *same string*, which is trivially equal, and then multiplies by `frame_transform(pose_frame)⁻¹ * frame_transform(tip)` — the same transform twice, so identity. The two branches compute the same query. The fast path is kept because upstream has it in the same position (`robot_state.cpp:1957-1990`) and it avoids two transform lookups per tip; it is not kept because anything depends on it. No test can be written that fails when it is removed. |
