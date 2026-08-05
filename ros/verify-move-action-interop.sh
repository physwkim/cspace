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
# Two legs, because they answer different questions:
#
#   Leg A (`ros2 action send_goal`, one container, the ros-dev image). Does
#     the node serve an action named `/move_action` that accepts a goal,
#     publishes upstream's `PLANNING` feedback, and terminates it with the
#     exact `MoveItErrorCodes` this port's handler builds? Cheap, no extra
#     image, and it can drive every boundary of the handler's conversion
#     branch, including the ones no C++ client can spell (see the table at
#     leg A below). Each goal's reply is captured in its own file, so an
#     assertion names the goal it is about -- three of the five boundaries now
#     answer with the same code, and a file-wide `grep` could not tell them
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
# Leg B needs `moveit_ros_planning_interface`, which means the oracle image
# (see ros/move_group_interface_probe/Dockerfile for why that and not the base
# image). When the oracle image is not built, leg B SKIPs in the loud shape
# tools/ci/verify-mpr-vs-epa.sh established -- naming what was not run and the
# command that would run it -- rather than passing quietly, because a silent
# skip is exactly the failure mode this whole file exists to close.
#
# What these legs still do NOT check:
#
#   - Nothing plans. Both legs assert an error, because there is no
#     `moveit_planning::pipeline::Planner` in this workspace to call
#     (D8/§140.3). When one lands, the expected strings below change, and that
#     is intended: the gate is pinned to what the node answers today, so the
#     first change to that answer has to come here. It already worked once --
#     these legs asserted `-16` and a "start_state is not representable"
#     message until `PlanningRequest` grew its start-state field, and both
#     assertions had to move with it.
#   - **Neither leg can see whether a start state's values actually landed.**
#     A goal carrying `joint_state: {name: [j1], position: [0.25]}` and a goal
#     carrying no start state at all get the same reply from this node, because
#     nothing plans and no trajectory comes back: the only observable is the
#     error, and both convert. So a conversion that dropped the values, or
#     paired them with the wrong names, is invisible here. That is checked
#     in-process instead, by
#     `each_start_state_position_is_carried_against_its_own_joint_name` and
#     `round_trip_start_state_through_msg` in ros/moveit-ros/src/planning.rs
#     and by `an_overlay_pairs_each_value_with_its_own_name` in
#     crates/moveit-planning/src/start_state.rs. What leg A *can* see is the
#     two arrays' roles: the length-mismatch boundary asserts the exact counts,
#     so a conversion that read `position` where `name` belongs still reddens.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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
# `plan_kinematic_path_server.rs` builds them, and nothing generates them from
# there: this is a hand-kept copy, deliberately, because the point of the gate
# is to notice when the answer changes. Deriving it from the source it checks
# would make it assert only that the source equals itself.
NO_PLANNER_MSG="no moveit_planning::pipeline::Planner to call yet"
SOURCE_STRING="moveit-ros/move_action"
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
  bash -c "cargo build --bin plan_kinematic_path_server" >&2

# One file per goal, mounted at /out, rather than one merged stream. Three of
# the five boundaries below now answer with the same `val`, so a file-wide
# `grep` could no longer say which goal produced which reply -- it would pass
# with four of the five goals never sent. The node's stderr is a sixth file
# for the same reason it was separate before: upstream's plan-only warning goes
# there, and asserting on it out of a merged stream would pass just as well if
# `ros2` had printed it.
leg_a_dir="$(mktemp -d)"
trap 'rm -rf "$leg_a_dir"' EXIT

# The five `start_state` shapes this port's conversion distinguishes. The first
# three convert (and so reach the no-planner arm); the last two do not, each
# with its own message. A C++ `MoveGroupInterface` can only ever produce the
# second and third -- rows 1, 4 and 5 exist only because leg A drives the
# action interface directly.
#
# `|| true` on each `send_goal`: a goal that ends ABORTED -- the terminal state
# this port always reaches -- is a non-zero exit for the CLI, and that is the
# expected outcome, not a failure. What must not happen is the reply going
# missing, and the assertions below catch that.
docker run --rm -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -v "$leg_a_dir:/out" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  ./target/debug/plan_kinematic_path_server '"$URDF $SRDF"' 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  ros2 action list >/out/actions.txt 2>&1

  send() { # <name> <goal yaml> [extra send_goal args...]
    local name="$1" goal="$2"
    shift 2
    timeout 25 ros2 action send_goal "$@" /move_action moveit_msgs/action/MoveGroup \
      "$goal" >"/out/$name.txt" 2>&1 || true
  }

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
# rejecting one shape cannot hide behind the four it still accepts.
for goal in unset empty-diff override length-mismatch multi-dof; do
  assert_has "leg A/$goal goal acceptance" "Goal accepted with ID:" "$leg_a_dir/$goal.txt"
  # Both replies carry this node's own `source`, which is what separates an
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
# handler falls through to the arm that stands in for upstream's
# null-pipeline `FAILURE` (move_action_capability.cpp:207-211). `assert_lacks`
# on the rejection code as well as `assert_has` on 99999: without it, a reply
# carrying both (an impossible message, but not one this file could otherwise
# rule out) would pass.
for goal in unset empty-diff override; do
  assert_has "leg A/$goal converted code" "val: 99999" "$leg_a_dir/$goal.txt"
  assert_has "leg A/$goal converted message" "$NO_PLANNER_MSG" "$leg_a_dir/$goal.txt"
  assert_lacks "leg A/$goal converted" "val: -16" "$leg_a_dir/$goal.txt"
done

# Boundary 4: the name/position length rule. The counts are part of the
# assertion, not decoration -- a conversion that read `position` where `name`
# belongs still rejects this goal, and only the two numbers say which array
# was which.
assert_has "leg A/length-mismatch code" "val: -16" "$leg_a_dir/length-mismatch.txt"
assert_has "leg A/length-mismatch message" "$LENGTH_MSG" "$leg_a_dir/length-mismatch.txt"

# Boundary 5: a start_state field with no core representation at all, which is
# a different rejection with a different owner from boundary 4's.
assert_has "leg A/multi-dof code" "val: -16" "$leg_a_dir/multi-dof.txt"
assert_has "leg A/multi-dof message" "$MULTI_DOF_MSG" "$leg_a_dir/multi-dof.txt"

echo "OK leg A: /move_action accepted goals over DDS, published PLANNING feedback,"
echo "OK leg A: converted three start_state shapes to 99999/$NO_PLANNER_MSG,"
echo "OK leg A: and rejected two with -16/$LENGTH_MSG and -16/$MULTI_DOF_MSG"

###############################################################################
# Leg B -- upstream's C++ MoveGroupInterface, two containers
###############################################################################
echo "=== move_action (leg B: upstream C++ MoveGroupInterface) ==="

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
ORACLE_STAMP="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
ORACLE_IMAGE="${ORACLE_IMAGE:-$(oracle_image_tag "$ORACLE_STAMP")}"
PROBE_IMAGE="${PROBE_IMAGE:-moveit-rs/move-group-interface-probe:${ORACLE_STAMP:0:16}}"

if ! docker image inspect "$ORACLE_IMAGE" >/dev/null 2>&1; then
  echo "SKIP $ORACLE_IMAGE not built -- upstream's C++ MoveGroupInterface was not run"
  echo "SKIP against /move_action, so Phase 9's completion condition is unmeasured by"
  echo "SKIP this run and leg A above stands alone: it drives the action interface"
  echo "SKIP directly and cannot see whether the real client reaches it."
  echo "SKIP this is not a pass; build it with: sg docker -c tools/moveit-oracle/build.sh"
  exit 0
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

teardown() {
  docker rm -f "$NODE_CTR" >/dev/null 2>&1 || true
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
  ./target/debug/plan_kinematic_path_server "$URDF" "$SRDF" >/dev/null
sleep 3

leg_b_out="$(mktemp)"
teardown() {
  docker rm -f "$NODE_CTR" >/dev/null 2>&1 || true
  docker network rm "$NET" >/dev/null 2>&1 || true
  cleanup_ctx
  rm -rf "$leg_a_dir"
  rm -f "$leg_b_out"
}

# Both start-state spellings an unmodified client can produce. `plan()` ships
# the constructor's empty diff (`is_diff = true`) by default; `setStartState`
# replaces it with a fully specified state (`is_diff = false`, but
# `joint_state.name` populated). They convert to the two *different* variants of
# `moveit_planning::StartState` -- `CurrentState` and `Overriding` -- which is
# why both modes stay on the gate: one run cannot cover both variants, and the
# variant a real client picks is decided by whether it called `setStartState`.
for mode in default-start explicit-start; do
  : >"$leg_b_out"
  timeout 120 docker run --rm --network "$NET" \
    -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
    -v "$REPO_ROOT:/repo" --entrypoint bash "$PROBE_IMAGE" -c "
      source /opt/ros/\$ROS_DISTRO/setup.bash
      source /ws/install/setup.bash
      ros2 run move_group_interface_probe move_group_interface_probe \
        $URDF $SRDF arm $mode
    " >"$leg_b_out" 2>&1 || true

  # Printed on success too, in the `/plan_kinematic_path` leg's shape: a gate
  # whose measurement is only visible when it fails leaves the operator with
  # no way to see what "passed" meant.
  grep '^PROBE ' "$leg_b_out" || true

  assert_has "leg B/$mode model" "PROBE constructed" "$leg_b_out"
  assert_has "leg B/$mode mode" "PROBE mode=$mode" "$leg_b_out"

  # The one assertion that separates "the client reached this node" from "the
  # client gave up locally". `move_group_interface.cpp:659-663` returns
  # FAILURE with both strings empty when no action server is up, and FAILURE is
  # the same 99999 this node now answers -- so `val` alone no longer says
  # anything here and `source` carries the whole discrimination. The
  # `assert_lacks` below is what makes that explicit rather than implied.
  assert_has "leg B/$mode round trip" "PROBE plan val=99999 source='$SOURCE_STRING'" "$leg_b_out"
  assert_lacks "leg B/$mode client-local failure" "PROBE plan val=99999 source=''" "$leg_b_out"

  # The message is what moved: both spellings used to stop at the conversion
  # (`val=-16`, "PlanningRequest has no start-state field"). Both now convert,
  # so the first thing this port cannot do for an unmodified client is no
  # longer representing the request -- it is having no planner to run.
  #
  # No companion `assert_lacks "start_state"` here. It would read as the
  # negative half of this pair, but the node builds exactly one message per
  # reply, so any regression that puts `start_state` back into it also fails
  # the two assertions above -- the negative could never be the line that
  # fires, only a spurious red when some unrelated rclcpp log mentions the
  # field. A check that cannot fail for the defect it names is not a check.
  assert_has "leg B/$mode message" "$NO_PLANNER_MSG" "$leg_b_out"

  assert_has "leg B/$mode trajectory" "PROBE points=0 multi_dof_points=0" "$leg_b_out"
  assert_has "leg B/$mode verdict" "PROBE verdict=NO_VALID_TRAJECTORY" "$leg_b_out"
done

echo "OK leg B: upstream's unmodified MoveGroupInterface::plan() reached /move_action over"
echo "OK leg B: DDS in both start-state spellings, both converted, and the reply that"
echo "OK leg B: came back is this node's own no-planner FAILURE, not a start-state refusal"
echo "OK move_action: both legs passed"
