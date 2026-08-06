#!/bin/bash
# The `/move_action` legs of ros/verify-ros-interop.sh, which calls this as
# its last step. Split out rather than inlined there because leg B below
# orchestrates three containers and a docker network, and because these are
# the only checks in this repo that run upstream's own C++ client.
#
# PORTING-PLAN.md §250 measured this round trip by hand and recorded the
# result in prose. Prose is not a gate: the round that added `/move_action`
# left ros/verify-ros-interop.sh with no `move_action` string in it, so the
# gate passed on the very tree that introduced the action server and would
# have gone on passing through any later change to it. This file is what makes
# the measurement re-run.
#
# Three legs, because they answer different questions:
#
#   Leg A (`ros2 action send_goal`, one container, the ros-dev image). Does
#     the node serve an action named `/move_action` that accepts a goal,
#     publishes upstream's `PLANNING` feedback, and terminates it with the
#     exact `MoveItErrorCodes` this port's handler builds? Cheap, no extra
#     image, and it can drive every arm of the handler, including the ones no
#     C++ client can spell (see the table at leg A below): a goal that plans, a
#     goal that converts and then fails inside the planner, and three
#     `start_state` shapes. Each goal's reply is captured in its own file, so an
#     assertion names the goal it is about -- the six replies carry only three
#     distinct `val`s between them, and a file-wide `grep` could not tell them
#     apart.
#
#   Leg B (upstream's `moveit::planning_interface::MoveGroupInterface`, two
#     containers). Does the client Phase 9's completion condition names --
#     unmodified, compiled from the pinned moveit2 sha -- actually reach this
#     node and get this node's answer back? Leg A cannot answer that: it
#     drives the action interface directly, so it would keep passing if the
#     endpoint were correct but unreachable by the real client (a rejected
#     goal callback, a name the client's `rclcpp::names::append` does not
#     resolve to, a `MoveGroup.action` the client's moveit_msgs disagrees
#     with). Leg B is the measurement, leg A is the fine-grained diagnosis
#     of why leg B failed.
#
#   Leg C (the same client and containers as leg B, calling
#     `computeCartesianPath`). Does `/compute_cartesian_path` answer the same
#     client? It is a service and not the action, so leg B's result says
#     nothing about it -- this node served `/move_action` for a whole round
#     with no Cartesian endpoint at all. Its in-process counterpart is
#     `ros/moveit-ros/src/cartesian_path.rs`'s test module, which can drive
#     the arms no unmodified client can reach: the three jump thresholds are
#     not parameters of the client's signature at the pinned sha, so leg C
#     structurally cannot set them.
#
# Legs B and C need `moveit_ros_planning_interface`, which means the oracle image
# (see ros/move_group_interface_probe/Dockerfile for why that and not the base
# image). When the oracle image is not built, leg B SKIPs in the loud shape
# tools/ci/verify-mpr-vs-epa.sh established -- naming what was not run and the
# command that would run it -- rather than passing quietly, because a silent
# skip is exactly the failure mode this whole file exists to close.
#
# What these legs still do NOT check:
#
#   - **Neither leg can see whether a start state's values actually landed.**
#     A goal carrying `joint_state: {name: [j1], position: [0.25]}` and a goal
#     carrying no start state at all get the same reply from this node, because
#     neither names a group and both therefore fail in the planner before any
#     trajectory exists: the only observable is the error, and both convert. So
#     a conversion that dropped the values, or paired them with the wrong names,
#     is invisible here. That is checked in-process instead, by
#     `each_start_state_position_is_carried_against_its_own_joint_name` and
#     `round_trip_start_state_through_msg` in ros/moveit-ros/src/planning.rs
#     and by `an_overlay_pairs_each_value_with_its_own_name` in
#     crates/moveit-planning/src/start_state.rs. What leg A *can* see is the
#     two arrays' roles: the length-mismatch boundary asserts the exact counts,
#     so a conversion that read `position` where `name` belongs still reddens.
#   - The trajectory's *content*. Leg A asserts the planned goal's reply names
#     the joint it moves; that it moves it to the requested position is checked
#     in-process, by `the_plan_only_arm_reaches_rrt_connect_and_gets_a_trajectory`
#     in ros/moveit-ros/src/move_group.rs.
#
# The strings below are pinned to what the node answers today, so the first
# change to that answer has to come here. That has now worked twice: these legs
# asserted `-16` and a "start_state is not representable" message until
# `PlanningRequest` grew its start-state field (§256), and `99999` with a
# "no Planner to call yet" message until D8 gave the node a planner.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"
URDF=/repo/ros/fixtures/one_joint.urdf
SRDF=/repo/ros/fixtures/one_joint.srdf

# Seven worktrees share this docker daemon, and both legs assert on replies
# they matched by content, not by sender. A domain id of this run's own keeps
# a concurrently-running copy of this gate off the same graph; leg B adds a
# network of its own on top, because two containers have to find each other
# there and the default bridge is shared with everything else on the daemon.
# Derived from `$$` rather than `$RANDOM` so a failing run can be reproduced
# with the domain id printed in its own output.
DOMAIN_ID="${ROS_DOMAIN_ID:-$((($$ % 100) + 1))}"
echo "move_action legs: ROS_DOMAIN_ID=$DOMAIN_ID"

# The exact strings this port answers with. One definition, asserted by both
# legs, so leg A and leg B cannot drift into asserting different things about
# the same handler.
#
# `move_group.rs` builds them, and nothing generates them from
# there: this is a hand-kept copy, deliberately, because the point of the gate
# is to notice when the answer changes. Deriving it from the source it checks
# would make it assert only that the source equals itself.
# The planner's own failure, not the node's: `plan_only` wraps whatever
# `moveit_planning::generate_plan` returns, and that names the planner that
# ran. Asserting the planner's name here is what distinguishes "reached
# rrt_connect and it said no" from "found no planner at all", which this port
# reports with a different sentence entirely.
#
# The doubled apostrophes are not a typo. The node's message is
# `planner 'rrt_connect' failed: ...`; `ros2 action send_goal` prints the
# result as YAML, and a YAML single-quoted scalar escapes an apostrophe by
# doubling it. The `/plan_kinematic_path` leg in ros/verify-ros-interop.sh
# asserts the same sentence undoubled because `ros2 service call` prints a
# Python repr instead -- same node, same string, two renderings.
PLANNER_FAILED_MSG="planner ''rrt_connect'' failed: unknown joint model group"
SOURCE_STRING="moveit-ros/move_action"
# Leg C's own, from `moveit_ros::cartesian_path::SOURCE`. Pinned separately
# from `SOURCE_STRING` on purpose: the field's whole job is to say which
# endpoint answered, so a leg that accepted either string would be blind to
# the one mix-up it exists to catch.
CARTESIAN_SOURCE_STRING="moveit-ros/compute_cartesian_path"
# The two `start_state` shapes the conversion still rejects, now that a
# representable one is carried instead of refused. Both are structural: they
# name what the wire holds and no robot model is consulted, which is why they
# are the conversion's to report and not `StartState::apply_to`'s.
LENGTH_MSG="has 2 name(s) but 1 position(s)"
MULTI_DOF_MSG="start_state.multi_dof_joint_state has no core representation"

fail() {
  echo "FAIL $*" >&2
  exit 1
}

# `if ! grep` rather than `grep || fail`: under this file's own `set -e` a
# `grep ... && fail` compound returns grep's 1 when the needle is absent and
# aborts the script at that line, so the negative assertion would report
# nothing at all -- the shape ros/verify-ros-interop.sh's own `test_status`
# and `|| true` comments already record twice.
assert_has() { # <what> <fixed string> <file>
  if ! grep -qF -- "$2" "$3"; then
    echo "--- captured output ---" >&2
    cat "$3" >&2
    fail "$1: expected this string and did not find it: $2"
  fi
}

assert_lacks() { # <what> <fixed string> <file>
  if grep -qF -- "$2" "$3"; then
    echo "--- captured output ---" >&2
    cat "$3" >&2
    fail "$1: this string must not appear and did: $2"
  fi
}

# Whole line, for `ros2 action list`'s one-name-per-line output. A substring
# match there would accept `/move_action_2` as `/move_action` -- and a renamed
# endpoint that still contains the old name as a prefix is precisely the
# regression this assertion is for.
assert_line() { # <what> <exact line> <file>
  if ! grep -qxF -- "$2" "$3"; then
    echo "--- captured output ---" >&2
    cat "$3" >&2
    fail "$1: expected this exact line and did not find it: $2"
  fi
}

###############################################################################
# Leg A -- ros2 action send_goal, in the ros-dev image
###############################################################################
echo "=== move_action (leg A: ros2 action send_goal) ==="

# `cargo build` here and not in the container command below: leg B reuses the
# binary this produces, out of the same bind-mounted target/ directory, so it
# is built once for both legs.
docker run --rm -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" \
  bash -c "cargo build --bin move_group" >&2

# One file per goal, mounted at /out, rather than one merged stream. The six
# boundaries below carry only three distinct `val`s between them, so a file-wide
# `grep` could no longer say which goal produced which reply -- it would pass
# with five of the six goals never sent. The node's stderr is a seventh file
# for the same reason it was separate before: upstream's plan-only warning goes
# there, and asserting on it out of a merged stream would pass just as well if
# `ros2` had printed it.
leg_a_dir="$(mktemp -d)"
trap 'rm -rf "$leg_a_dir"' EXIT

# One goal that plans, then the five `start_state` shapes this port's
# conversion distinguishes: three that convert (and then fail in the planner,
# since none of them names a group) and two that do not, each with its own
# message. A C++ `MoveGroupInterface` can only ever produce the empty-diff and
# override shapes -- the other four rows exist only because leg A drives the
# action interface directly.
#
# `|| true` on each `send_goal`: a goal that ends ABORTED is a non-zero exit
# for the CLI, and for five of the six that is the expected outcome, not a
# failure. The sixth is expected to SUCCEED, and carries the same `|| true` for
# a different reason -- without it a regression there would abort this
# container script before the other five goals ran, and the assertions below
# would report five missing replies instead of the one that actually changed.
docker run --rm -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -v "$leg_a_dir:/out" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  ./target/debug/move_group '"$URDF $SRDF"' 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  ros2 action list >/out/actions.txt 2>&1

  send() { # <name> <goal yaml> [extra send_goal args...]
    local name="$1" goal="$2"
    shift 2
    timeout 40 ros2 action send_goal "$@" /move_action moveit_msgs/action/MoveGroup \
      "$goal" >"/out/$name.txt" 2>&1 || true
  }

  # A group and a goal: the only goal here that a planner can answer. Every
  # other row leaves `group_name` empty, so this is the one that separates
  # "reached the planner" from "planned".
  send planned \
    "{request: {group_name: arm, goal_constraints: [{joint_constraints: [{joint_name: j1, position: 0.5, tolerance_above: 0.001, tolerance_below: 0.001, weight: 1.0}]}]}, planning_options: {plan_only: true}}"

  # No start_state at all, and `plan_only` left false so the node also emits
  # the upstream plan-only warning. Converts.
  send unset "{}" --feedback

  # The empty diff installed by the MoveGroupInterface constructor
  # (`setStartStateToCurrentState`, move_group_interface.cpp:434-439). This is
  # the exact message that used to be rejected. Converts, as CurrentState.
  send empty-diff \
    "{request: {start_state: {is_diff: true}}, planning_options: {plan_only: true}}"

  # A populated overlay: one named joint with a position. Converts, as an
  # override -- the shape `setStartState(const RobotState&)` produces, minus
  # its `is_diff: false`.
  send override \
    "{request: {start_state: {is_diff: true, joint_state: {name: [j1], position: [0.25]}}}, planning_options: {plan_only: true}}"

  # Two names, one position: the wire convention violation that upstream
  # rejects in jointStateToRobotStateImpl (conversions.cpp:64-69). Does not
  # convert.
  send length-mismatch \
    "{request: {start_state: {joint_state: {name: [j1, j2], position: [0.25]}}}, planning_options: {plan_only: true}}"

  # A multi-DOF joint: representable on the wire, not in this port. Does not
  # convert.
  send multi-dof \
    "{request: {start_state: {multi_dof_joint_state: {joint_names: [virtual_joint]}}}, planning_options: {plan_only: true}}"

  cp /tmp/node.stderr /out/node.stderr
'

# The wire name, unqualified in upstream (`move_group::MOVE_ACTION`,
# capability_names.hpp:52) and resolved by the client through
# `rclcpp::names::append`. A rename reddens here first, with its own message,
# before the reply assertions fail for a reason that would read as "wrong
# error code".
assert_line "leg A action name" "/move_action" "$leg_a_dir/actions.txt"

# Upstream's goal callback is a constant ACCEPT (move_action_capability.cpp:
# 70-74) -- there is no rejection branch to port, so a rejected goal is this
# port having grown one. Asserted on every goal, so a handler that started
# rejecting one shape cannot hide behind the five it still accepts.
for goal in planned unset empty-diff override length-mismatch multi-dof; do
  assert_has "leg A/$goal goal acceptance" "Goal accepted with ID:" "$leg_a_dir/$goal.txt"
  # Every reply carries this node's own `source`, which is what separates an
  # answer from this port from an answer built anywhere else.
  assert_has "leg A/$goal source" "source: $SOURCE_STRING" "$leg_a_dir/$goal.txt"
done

# setMoveState(PLANNING, goal_) at move_action_capability.cpp:89. Only the
# `--feedback` goal subscribes to it.
assert_has "leg A PLANNING feedback" "state: PLANNING" "$leg_a_dir/unset.txt"

# The warning at move_action_capability.cpp:98-102, reached because
# `allow_trajectory_execution_` is false in this port and the `unset` goal
# leaves `plan_only` false.
assert_has "leg A plan_only warning" \
  "not allowed to execute trajectories but the goal request has plan_only set to false" \
  "$leg_a_dir/node.stderr"

# Boundaries 1-3: every start_state this port can represent converts, so the
# handler reaches the planner -- which rejects all three, because none of them
# names a group. Upstream reports its own unsolved-plan arm with the same
# `FAILURE` it reports a null pipeline with
# (move_action_capability.cpp:207-211,218-227). `assert_lacks` on the rejection
# code as well as `assert_has` on 99999: without it, a reply carrying both (an
# impossible message, but not one this file could otherwise rule out) would
# pass.
for goal in unset empty-diff override; do
  assert_has "leg A/$goal converted code" "val: 99999" "$leg_a_dir/$goal.txt"
  assert_has "leg A/$goal converted message" "$PLANNER_FAILED_MSG" "$leg_a_dir/$goal.txt"
  assert_lacks "leg A/$goal converted" "val: -16" "$leg_a_dir/$goal.txt"
done

# Boundary 4: the goal that plans. `SUCCEEDED` rather than the error code
# alone, because it is the only thing that separates the two arms of the
# handler's terminal branch (src/bin/move_group.rs, upstream's `:113-124`): a
# `val: 1` result delivered through `abort` would satisfy a code assertion and
# still be the wrong terminal state. `- j1` is the planned trajectory naming the
# joint it moves; an empty `planned_trajectory` has no joint_names at all.
assert_has "leg A/planned terminal state" "Goal finished with status: SUCCEEDED" "$leg_a_dir/planned.txt"
assert_has "leg A/planned code" "val: 1" "$leg_a_dir/planned.txt"
assert_has "leg A/planned trajectory" "- j1" "$leg_a_dir/planned.txt"

# Boundary 5: the name/position length rule. The counts are part of the
# assertion, not decoration -- a conversion that read `position` where `name`
# belongs still rejects this goal, and only the two numbers say which array
# was which.
assert_has "leg A/length-mismatch code" "val: -16" "$leg_a_dir/length-mismatch.txt"
assert_has "leg A/length-mismatch message" "$LENGTH_MSG" "$leg_a_dir/length-mismatch.txt"

# Boundary 6: a start_state field with no core representation at all, which is
# a different rejection with a different owner from boundary 5's.
assert_has "leg A/multi-dof code" "val: -16" "$leg_a_dir/multi-dof.txt"
assert_has "leg A/multi-dof message" "$MULTI_DOF_MSG" "$leg_a_dir/multi-dof.txt"

echo "OK leg A: /move_action accepted goals over DDS, published PLANNING feedback,"
echo "OK leg A: planned one to SUCCEEDED, answered three converted start_state shapes"
echo "OK leg A: with 99999/$PLANNER_FAILED_MSG, and rejected two with"
echo "OK leg A: -16/$LENGTH_MSG and -16/$MULTI_DOF_MSG"

###############################################################################
# Leg B -- upstream's C++ MoveGroupInterface, two containers
###############################################################################
echo "=== move_action (leg B: upstream C++ MoveGroupInterface) ==="

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
ORACLE_STAMP="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
ORACLE_IMAGE="${ORACLE_IMAGE:-$(oracle_image_tag "$ORACLE_STAMP")}"
PROBE_IMAGE="${PROBE_IMAGE:-moveit-rs/move-group-interface-probe:${ORACLE_STAMP:0:16}}"

# Exit 3, not 0, and not 1: three outcomes have to stay apart here. 1 is an
# assertion that ran and failed; 0 is both legs measured; 3 is leg A measured
# and leg B not run at all. Returning 0 for the third made
# `ros/verify-ros-interop.sh` print `all gates passed` over a run in which
# nothing had driven the real client -- the exact reading this block's own
# "this is not a pass" text was written to prevent, and could not, because the
# status the caller branches on said otherwise. The caller maps 3 to a summary
# line that names the skip; see the end of that file.
if ! docker image inspect "$ORACLE_IMAGE" >/dev/null 2>&1; then
  echo "SKIP $ORACLE_IMAGE not built -- upstream's C++ MoveGroupInterface was not run"
  echo "SKIP against /move_action, so Phase 9's completion condition is unmeasured by"
  echo "SKIP this run and leg A above stands alone: it drives the action interface"
  echo "SKIP directly and cannot see whether the real client reaches it."
  echo "SKIP this is not a pass; build it with: sg docker -c tools/moveit-oracle/build.sh"
  exit 3
fi

# Unconditional: the `--packages-up-to` layer keys only on ORACLE_IMAGE and the
# probe layer only on the copied sources, so docker's layer cache makes a
# no-change run a few seconds and a probe edit rebuild just the probe.
CTX="$(mktemp -d "$REPO_ROOT/.probe-ctx.XXXXXX")"
cleanup_ctx() { rm -rf "$CTX"; }
trap 'rm -rf "$leg_a_dir"; cleanup_ctx' EXIT
cp -R "$REPO_ROOT/ros/move_group_interface_probe" "$CTX/move_group_interface_probe"
cp "$REPO_ROOT/ros/move_group_interface_probe/Dockerfile" "$CTX/Dockerfile"
docker build \
  --build-arg "ORACLE_IMAGE=$ORACLE_IMAGE" \
  -t "$PROBE_IMAGE" \
  -f "$CTX/Dockerfile" \
  "$CTX"

NET="moveit-rs-move-action-$$"
NODE_CTR="moveit-rs-move-action-node-$$"

PROBE_CTR="moveit-rs-move-action-probe-$$"

teardown() {
  docker rm -f "$NODE_CTR" "$PROBE_CTR" >/dev/null 2>&1 || true
  docker network rm "$NET" >/dev/null 2>&1 || true
  cleanup_ctx
  rm -rf "$leg_a_dir"
}
trap teardown EXIT

docker network create "$NET" >/dev/null

# Detached, not backgrounded: the probe runs in a second container and both
# have to be up at once. `--rm` plus the trap above means neither survives a
# failing assertion below.
docker run -d --rm --name "$NODE_CTR" --network "$NET" \
  -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" \
  ./target/debug/move_group "$URDF" "$SRDF" >/dev/null
sleep 3

leg_b_out="$(mktemp)"
teardown() {
  docker rm -f "$NODE_CTR" "$PROBE_CTR" >/dev/null 2>&1 || true
  docker network rm "$NET" >/dev/null 2>&1 || true
  cleanup_ctx
  rm -rf "$leg_a_dir"
  rm -f "$leg_b_out"
}

# One probe run, bounded so that a probe which never returns reddens this gate
# instead of stalling it.
#
# `timeout 120 docker run` does not bound it. On expiry `timeout` sends
# SIGTERM to the docker *client*, which forwards the signal to the container
# and then waits for the container to exit -- and a probe blocked inside
# upstream's client does not exit, because
# `MoveGroupInterfaceImpl::computeCartesianPath` has no timeout on
# `future_response.get()` (`move_group_interface.cpp:893-896`) and neither has
# the service call under it. Measured while building leg C: with the node's
# service renamed away, `timeout 120 docker run` was still running 31 minutes
# later. The container name plus `docker rm -f` is the one signal it cannot
# ignore; `-k 5` is what stops `timeout` itself from waiting forever on the
# client it just signalled.
run_probe() {  # <mode> <output-file>
  : >"$2"
  timeout -k 5 120 docker run --rm --name "$PROBE_CTR" --network "$NET" \
    -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
    -v "$REPO_ROOT:/repo" --entrypoint bash "$PROBE_IMAGE" -c "
      source /opt/ros/\$ROS_DISTRO/setup.bash
      source /ws/install/setup.bash
      ros2 run move_group_interface_probe move_group_interface_probe \
        $URDF $SRDF arm $1
    " >"$2" 2>&1 || true
  docker rm -f "$PROBE_CTR" >/dev/null 2>&1 || true
}

# Both start-state spellings an unmodified client can produce. `plan()` ships
# the constructor's empty diff (`is_diff = true`) by default; `setStartState`
# replaces it with a fully specified state (`is_diff = false`, but
# `joint_state.name` populated). They convert to the two *different* variants of
# `moveit_planning::StartState` -- `CurrentState` and `Overriding` -- which is
# why both modes stay on the gate: one run cannot cover both variants, and the
# variant a real client picks is decided by whether it called `setStartState`.
for mode in default-start explicit-start; do
  run_probe "$mode" "$leg_b_out"

  # Printed on success too, in the `/plan_kinematic_path` leg's shape: a gate
  # whose measurement is only visible when it fails leaves the operator with
  # no way to see what "passed" meant.
  grep '^PROBE ' "$leg_b_out" || true

  assert_has "leg B/$mode model" "PROBE constructed" "$leg_b_out"
  assert_has "leg B/$mode mode" "PROBE mode=$mode" "$leg_b_out"

  # The one assertion that separates "the client reached this node" from "the
  # client gave up locally". `move_group_interface.cpp:659-663` returns
  # FAILURE with both strings empty when no action server is up, and the code
  # this node answers is now SUCCESS -- so a bare `val=1` could not have come
  # from that local path, but `source` is still asserted rather than dropped:
  # it is what says the SUCCESS was built *here*, and it is the reason
  # `src/bin/move_group.rs` stamps the field on the success arm too.
  assert_has "leg B/$mode round trip" "PROBE plan val=1 source='$SOURCE_STRING'" "$leg_b_out"
  assert_lacks "leg B/$mode client-local failure" "PROBE plan val=99999 source=''" "$leg_b_out"

  # `MoveGroupInterface`'s constructor leaves `active_target_ = JOINT`
  # (move_group_interface.cpp:156), so `constructMotionPlanRequest` fills
  # `goal_constraints[0]` from `getTargetRobotState()` (`:1041-1046`) even
  # though this probe never sets a target. That is why an unmodified client
  # gets a plan out of a node with one planner registered.
  assert_lacks "leg B/$mode empty trajectory" "PROBE points=0 " "$leg_b_out"

  # Phase 9's condition is a *valid* trajectory, and the three clauses below are
  # what that decomposes into here, asserted one line each so a red gate names
  # which one broke instead of only that the conjunction did. All three are
  # graded by upstream's own `moveit_core` inside the probe, never by the node
  # that produced the trajectory.
  #
  # Collision-freeness is deliberately absent from this list. `one_joint.urdf`
  # declares no `<collision>` element and the node runs with no world, so every
  # trajectory is collision-free and an assertion on it would be one that cannot
  # fail; the probe prints `PROBE colliding=` with the object and geometry
  # counts beside it so that stays visible rather than being quietly implied by
  # the verdict.
  assert_has "leg B/$mode joint limits" "PROBE all_in_bounds=true" "$leg_b_out"
  assert_has "leg B/$mode goal reached" "PROBE goal_satisfied=true" "$leg_b_out"
  assert_has "leg B/$mode verdict" "PROBE verdict=VALID_TRAJECTORY_RECEIVED" "$leg_b_out"
done

echo "OK leg B: upstream's unmodified MoveGroupInterface::plan() reached /move_action over"
echo "OK leg B: DDS in both start-state spellings and got a real trajectory back from"
echo "OK leg B: this node, with source=$SOURCE_STRING naming the endpoint that built it"
echo "OK leg B: and upstream's own moveit_core graded every waypoint inside j1's limits"
echo "OK leg B: and the last one satisfying the goal_constraints the client itself sent."
echo "OK leg B: Collision-freeness is printed, not graded: one_joint.urdf has no"
echo "OK leg B: collision geometry, so no trajectory over it can collide."

# Leg C: `/compute_cartesian_path`, the same unmodified client and the same
# two containers, but a service rather than the action -- so nothing leg B
# asserted carries over. The node could serve `/move_action` exactly as
# measured above and not have this endpoint at all, which is the state this
# round found it in.
#
# It is a third `for` iteration rather than a fourth mode of the loop above
# because none of that loop's assertions apply: there is no `plan val=`, no
# `goal_constraints` (the client builds none for a Cartesian request,
# `move_group_interface.cpp:878-889` sets ten fields and that is not among
# them), and the source string is this endpoint's own.
run_probe cartesian "$leg_b_out"

grep '^PROBE ' "$leg_b_out" || true

assert_has "leg C model" "PROBE constructed" "$leg_b_out"
assert_has "leg C mode" "PROBE mode=cartesian" "$leg_b_out"

# Same discriminator as leg B and for the same reason: the client answers
# `-1.0` both for a real non-SUCCESS reply and for a call that reached no
# server (`move_group_interface.cpp:899-911`), so the pair (`val`, `source`)
# is what says this node built the answer. `$CARTESIAN_SOURCE_STRING` is not
# `$SOURCE_STRING` -- a reply carrying the action's source would mean the
# service reply was assembled by the wrong handler.
assert_has "leg C round trip" "PROBE cartesian val=1 source='$CARTESIAN_SOURCE_STRING'" "$leg_b_out"
assert_lacks "leg C client-local failure" "PROBE cartesian val=99999 source=''" "$leg_b_out"

# `fraction` is the client's own return value, so it is asserted at its exact
# value and not as "> 0": `j1` has no `<origin>` in one_joint.urdf, which makes
# every pose on the straight line to the requested one exactly reachable, so a
# partial answer here is a defect and not a property of the fixture.
assert_has "leg C fraction" "PROBE cartesian fraction=1" "$leg_b_out"
assert_lacks "leg C empty path" "PROBE cartesian points=0" "$leg_b_out"

# Both graded inside the probe by upstream's own moveit_core, never by the
# node that produced the path: every waypoint inside `j1`'s limits, and the
# last one placing the client's own end-effector link at the pose the client
# itself sent. A node that answered `fraction=1` for a path ending elsewhere
# fails the second.
assert_has "leg C joint limits" "PROBE cartesian all_in_bounds=true" "$leg_b_out"
assert_has "leg C pose reached" "PROBE cartesian reached=true" "$leg_b_out"
assert_has "leg C verdict" "PROBE cartesian verdict=FULL_CARTESIAN_PATH_RECEIVED" "$leg_b_out"

echo "OK leg C: upstream's unmodified MoveGroupInterface::computeCartesianPath() reached"
echo "OK leg C: /compute_cartesian_path over DDS and got fraction=1 back from this node,"
echo "OK leg C: with source=$CARTESIAN_SOURCE_STRING naming the endpoint that built it,"
echo "OK leg C: every waypoint inside j1's limits and the last one placing tip at the"
echo "OK leg C: pose the client sent, both graded by upstream's own moveit_core."
echo "OK move_action: all three legs passed"
