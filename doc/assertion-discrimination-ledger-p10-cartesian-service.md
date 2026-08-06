# Assertion-discrimination ledger — p10-cartesian-service (`ros/moveit-ros::cartesian_path`)

The seven sites the sweep's scanner emits for the
`/compute_cartesian_path` service binding. Six are substring tests on
`error_code.message` and one is an emptiness test on a response field.

This is a separate file from
`assertion-discrimination-ledger-p10-cartesian.md`, which covers
`moveit-kinematics::cartesian_interpolator` — the interpolator the service
calls, ported by a different panel. Same panel-name prefix, different
crate and different assertions.

## Method

Every row's evidence is a mutation applied to this worktree, run as
`cargo test --lib` inside `moveit-rs/ros-dev:latest` (nextest is not
installed in that image; 227 tests, 227 passing at baseline), and then
reverted, with the revert confirmed by `git diff --stat` reporting the
file clean. No row is justified by reading the code, and none by a bite
run in an earlier round.

`message.contains(x)` is the shape this ledger exists to be sceptical of:
it passes for *any* message containing `x`, so a bite that only reworded
the port's own prefix would not tell whether the assertion reads the port
or reads the error it wrapped. Each row below therefore names which of the
two carries the substring, and the two link-name rows are each other's
isolating mutation — the mutation that makes the message a constant equal
to one test's expected needle fails the *other* test alone.

## Sites

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `ros/moveit-ros/src/cartesian_path.rs:985` | `assert!(response.error_code.message.contains("base_link"), ...)` — the refusal for a group link no solver tip is rigidly connected to | `a_group_link_no_solver_tip_reaches_is_refused_rather_than_scored_zero` | discriminating | Bite C1d replaces the whole `Err` arm of the interpolator call with the constant `failure("no_such_link")`: this test alone fails, printing `got "no_such_link"`, and the sibling row for `an_unknown_link_name_is_refused_rather_than_crashing` stays green. The converse, C1c (`failure("base_link")`), leaves this test green and fails that sibling alone. So the assertion reads the link *this* request named, not a fixed word both refusals happen to contain. C1b (`failure("computing the Cartesian path failed")`) fails both, which locates the substring in the message rather than in the error code. |
| `ros/moveit-ros/src/cartesian_path.rs:1166` | `assert!(response.start_state.joint_state.name.is_empty(), "upstream never reaches ...")` | `no_waypoints_answers_success_with_an_untouched_response` | discriminating | Bite C6 gives the `Computed::NothingRequested` arm of `handle` a `start_state` with `joint_state.name = ["j1"]` and leaves `solution` default: this assertion fails and the `points.is_empty()` assertion in the same test body does not (the panic names the start_state line). C7, the mirror, leaves this one green. The two are a partition of the response, not one assertion counted twice. |
| `ros/moveit-ros/src/cartesian_path.rs:1170` | `assert!(response.solution.joint_trajectory.points.is_empty())` | `no_waypoints_answers_success_with_an_untouched_response` | discriminating | Bite C7 gives the same arm a `solution` with one trajectory point and leaves `start_state` default: this assertion fails (`assertion failed: response.solution.joint_trajectory.points.is_empty()`) and the `start_state` assertion above it does not. Under C6 it stays green. |
| `ros/moveit-ros/src/cartesian_path.rs:1260` | `assert!(response.error_code.message.contains("start_state"), ...)` | `an_unrepresentable_start_state_is_refused` | discriminating, carried by the cause | Bite C3b replaces `failure(format!("start_state is not representable: {e}"))` with the constant `failure("not representable")`: this test alone fails, `got "not representable"`. But C3c, which keeps `{e}` and drops only the port's own `start_state` prefix, leaves it **green** — so the substring this assertion reads comes from the `StartState::try_from` error, not from the wording here. The assertion is a statement about what reaches the client, which is what it should be; it is not a statement about this call site's format string, and rewording that string alone will not fail it. |
| `ros/moveit-ros/src/cartesian_path.rs:1291` | `assert!(response.error_code.message.contains("path_constraints"), ...)` | `an_unrepresentable_path_constraint_is_refused` | discriminating | Bite C4 keeps `{e}` and reworks only the prefix (`"the request is not representable: {e}"`): this test alone fails, printing `got "the request is not representable: no joint named \"not_a_joint\""`. Unlike `an_unrepresentable_start_state_is_refused`, the underlying error here names the joint and not the field, so the port's own prefix is what carries the field name, and a bite on that prefix is enough. |
| `ros/moveit-ros/src/cartesian_path.rs:1311` | `assert!(response.error_code.message.contains("no_such_link"), ...)` | `an_unknown_link_name_is_refused_rather_than_crashing` | discriminating | Bite C1c (`failure("base_link")`) fails this test alone, `got "base_link"`, while `a_group_link_no_solver_tip_reaches_is_refused_rather_than_scored_zero` stays green; C1d is the converse. See that row — the pair is one experiment. |
| `ros/moveit-ros/src/cartesian_path.rs:1350` | `assert!(response.error_code.message.contains(name), ...)` inside the four-field loop over `jump_threshold`, `prismatic_jump_threshold`, `revolute_jump_threshold`, `max_cartesian_speed` | `a_filter_this_service_does_not_apply_is_refused_rather_than_ignored` | discriminating | Bite C5b renames the other three entries of `compute`'s refusal table to `"jump_threshold"`, so every refusal reports itself as that field while still refusing at the same values: this test alone fails. The `assert_val` above it does not move under C5b, which is the point — the error code is `FAILURE` for all four either way, and only the substring test can say the response named the field the client actually set. |

## What no row here claims

None of these seven asserts a *fraction*. The fraction assertions in this
module (`== 1.0`, `> 0.0 && < 1.0`, `== 0.0`) are exact-value or
interval comparisons and the scanner does not emit them as coarse sites;
their discrimination is established by the guard bites B1–B17 recorded in
this round's report, not here.
