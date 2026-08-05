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
#     image, and it can drive *both* sides of the handler's one branch --
#     a default `start_state`, which converts, and a non-default one, which
#     does not. The C++ client can only ever produce the second.
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
# What these legs still do NOT check: nothing plans. Both legs assert an
# error, because there is no `moveit_planning::pipeline::Planner` in this
# workspace to call (D8/§140.3) and no start-state field on
# `moveit_planning::PlanningRequest` (§250.6). When either lands, the expected
# strings below change, and that is intended: the gate is pinned to what the
# node answers today, so the first change to that answer has to come here.
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
START_STATE_MSG="MotionPlanRequest.start_state is not representable"
NO_PLANNER_MSG="no moveit_planning::pipeline::Planner to call yet"
SOURCE_STRING="moveit-ros/move_action"

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

leg_a_out="$(mktemp)"
trap 'rm -f "$leg_a_out"' EXIT

# Node stderr is captured separately from the two goal replies: upstream's
# plan-only warning goes to the node's stderr, and asserting on it out of a
# merged stream would pass just as well if `ros2` printed it.
#
# `|| true` on the two `send_goal` calls: a goal that ends ABORTED -- which is
# the terminal state this port always reaches -- is a non-zero exit for the
# CLI, and that is the expected outcome here, not a failure. What must not
# happen is the reply going missing, and the assertions below catch that.
docker run --rm -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  ./target/debug/plan_kinematic_path_server '"$URDF $SRDF"' 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  echo "@@@ action list"
  ros2 action list

  echo "@@@ default start_state (converts; reaches the no-planner arm)"
  timeout 25 ros2 action send_goal --feedback /move_action \
    moveit_msgs/action/MoveGroup "{}" || true

  echo "@@@ non-default start_state (the only shape MoveGroupInterface sends)"
  timeout 25 ros2 action send_goal /move_action moveit_msgs/action/MoveGroup \
    "{request: {start_state: {is_diff: true}}, planning_options: {plan_only: true}}" || true

  echo "@@@ node stderr"
  cat /tmp/node.stderr
' >"$leg_a_out" 2>&1

# The wire name, unqualified in upstream (`move_group::MOVE_ACTION`,
# capability_names.hpp:52) and resolved by the client through
# `rclcpp::names::append`. A rename reddens here first, with its own message,
# before the reply assertions fail for a reason that would read as "wrong
# error code".
assert_line "leg A action name" "/move_action" "$leg_a_out"

# Upstream's goal callback is a constant ACCEPT (move_action_capability.cpp:
# 70-74) -- there is no rejection branch to port, so a rejected goal is this
# port having grown one.
assert_has "leg A goal acceptance" "Goal accepted with ID:" "$leg_a_out"

# setMoveState(PLANNING, goal_) at move_action_capability.cpp:89.
assert_has "leg A PLANNING feedback" "state: PLANNING" "$leg_a_out"

# The warning at move_action_capability.cpp:98-102, reached because
# `allow_trajectory_execution_` is false in this port and the `{}` goal leaves
# `plan_only` false.
assert_has "leg A plan_only warning" \
  "not allowed to execute trajectories but the goal request has plan_only set to false" \
  "$leg_a_out"

# Boundary 1: a default `start_state` converts, so the handler falls through to
# the arm that stands in for upstream's null-pipeline `FAILURE`
# (move_action_capability.cpp:207-211).
assert_has "leg A default-start_state code" "val: 99999" "$leg_a_out"
assert_has "leg A default-start_state message" "$NO_PLANNER_MSG" "$leg_a_out"

# Boundary 2: a non-default `start_state` does not convert.
assert_has "leg A non-default-start_state code" "val: -16" "$leg_a_out"
assert_has "leg A non-default-start_state message" "$START_STATE_MSG" "$leg_a_out"

# Both replies carry this node's own `source`, which is what separates an
# answer from this port from an answer built anywhere else.
assert_has "leg A source" "source: $SOURCE_STRING" "$leg_a_out"

echo "OK leg A: /move_action accepted goals over DDS, published PLANNING feedback,"
echo "OK leg A: and answered 99999/$NO_PLANNER_MSG and -16/$START_STATE_MSG"

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
trap 'rm -f "$leg_a_out"; cleanup_ctx' EXIT
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
  rm -f "$leg_a_out"
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
  rm -f "$leg_a_out" "$leg_b_out"
}

# Both start-state spellings an unmodified client can produce. `plan()` ships
# the constructor's empty diff (`is_diff = true`) by default; `setStartState`
# replaces it with a fully specified state (`is_diff = false`, but
# `joint_state.name` populated). Both are non-default `RobotState` messages, so
# both must land on the same rejection -- that is the invariant boundary, not
# two retellings of one scenario.
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
  # FAILURE with both strings empty when no action server is up; only a reply
  # that crossed DDS can carry this node's `source`.
  assert_has "leg B/$mode round trip" "PROBE plan val=-16 source='$SOURCE_STRING'" "$leg_b_out"
  assert_lacks "leg B/$mode client-local failure" "PROBE plan val=99999 source=''" "$leg_b_out"

  assert_has "leg B/$mode message" "$START_STATE_MSG" "$leg_b_out"
  assert_has "leg B/$mode trajectory" "PROBE points=0 multi_dof_points=0" "$leg_b_out"
  assert_has "leg B/$mode verdict" "PROBE verdict=NO_VALID_TRAJECTORY" "$leg_b_out"
done

echo "OK leg B: upstream's unmodified MoveGroupInterface::plan() reached /move_action over"
echo "OK leg B: DDS in both start-state spellings and got this node's own -16 back"
echo "OK move_action: both legs passed"
