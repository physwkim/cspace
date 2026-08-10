#!/bin/bash
# The `/execute_trajectory` leg of ros/verify-ros-interop.sh, which calls this
# as one of its steps.
#
# One leg, one container, `ros2 action send_goal` -- there is no leg B here
# and that is a measurement, not an omission. Upstream's own client reaches
# this endpoint from `execute`/`asyncExecute`
# (move_group_interface.hpp:732,741,750,759) and from nothing else; its
# constructor merely opens the client and discards
# `wait_for_action_server`'s return (move_group_interface.cpp:191-193). So a
# C++ probe that only constructs and plans cannot observe this server's
# ANSWER at all -- it can only observe how long the constructor took. That
# second measurement belongs to whoever times the constructor; what this file
# checks is the answer.
#
# What is asserted, and why each line is here rather than folded into one:
#
#   - the action is on the graph under upstream's exact unqualified name
#     (move_group::EXECUTE_ACTION_NAME, capability_names.hpp:45);
#   - the goal is ACCEPTED -- upstream's goal callback is a constant
#     ACCEPT_AND_EXECUTE (execute_trajectory_action_capability.cpp:79-81), so
#     a rejected goal is this port having grown a branch upstream does not
#     have;
#   - the terminal state is ABORTED, not SUCCEEDED. This is the assertion the
#     whole endpoint turns on: a `val: -4` delivered through `succeed` would
#     satisfy an error-code check and still tell the client the execution
#     worked;
#   - `val: -4` exactly, and not the codes a *real* backend would answer with.
#     A client branches on this number;
#   - `source` names this endpoint, so an answer built here is distinguishable
#     from one built by /move_action in the same node or by a client that
#     never reached the node;
#   - no feedback is published, matching the branch this port lands on
#     (upstream aborts at execute_trajectory_action_capability.cpp:94 before
#     any setExecuteTrajectoryState call);
#   - and the answer is the SAME for a goal carrying a real trajectory and
#     controller names as for an empty one. That pair is the check on the
#     module's "one condition, one code, no boundary where the rule changes"
#     claim: upstream's branch returns before either field is read, so a port
#     that quietly grew a per-goal special case would show up as two different
#     replies here and nowhere else.
#
# The strings are pinned to what the node answers today, so the first change
# to that answer has to come here -- the same rule
# ros/verify-move-action-interop.sh states at greater length, and for the same
# reason: deriving the expected strings from the source under test would make
# this assert only that the source equals itself.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"
URDF=/repo/ros/fixtures/one_joint.urdf
SRDF=/repo/ros/fixtures/one_joint.srdf

# Seven worktrees share this docker daemon, and the assertions below match
# replies by content rather than by sender. Derived from `$$` rather than
# `$RANDOM` so a failing run is reproducible from the id printed in its own
# output -- ros/verify-move-action-interop.sh records the full reasoning.
DOMAIN_ID="${ROS_DOMAIN_ID:-$((($$ % 100) + 1))}"
echo "=== execute_trajectory (ROS_DOMAIN_ID=$DOMAIN_ID) ==="

# The exact strings this port answers with, hand-kept: see the header.
# `moveit_ros::execute_trajectory` builds them.
SOURCE_STRING="cspace-ros/execute_trajectory"
# A distinctive fragment of NO_EXECUTION_BACKEND, not the whole sentence:
# `ros2 action send_goal` re-wraps long YAML scalars, so a whole-sentence
# fixed-string match would fail on line breaks the node did not put there.
# This fragment names the upstream branch, which is the part carrying the
# information -- three of upstream's six rows answer -4, and `message` is the
# only thing that says which one this is.
BRANCH_MSG="!trajectory_execution_manager_"

fail() {
  echo "FAIL $*" >&2
  exit 1
}

# `if ! grep`, not `grep || fail`: under `set -e` a `grep ... && fail`
# compound returns grep's 1 when the needle is absent and aborts the script at
# that line, so the assertion would report nothing at all.
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

# Whole line, for `ros2 action list`'s one-name-per-line output: a substring
# match accepts `/execute_trajectory_2` as `/execute_trajectory`, and a rename
# that keeps the old name as a prefix is exactly the regression this is for.
assert_line() { # <what> <exact line> <file>
  if ! grep -qxF -- "$2" "$3"; then
    echo "--- captured output ---" >&2
    cat "$3" >&2
    fail "$1: expected this exact line and did not find it: $2"
  fi
}

docker_cargo_run --rm -v "$REPO_ROOT:/repo" -w /repo/ros/cspace-ros "$IMAGE" \
  bash -c "cargo build --bin move_group" >&2

# One file per goal, as in ros/verify-move-action-interop.sh: the two replies
# are asserted to be *the same*, and a file-wide grep over a merged stream
# would pass with only one of the two goals ever sent.
out_dir="$(mktemp -d)"
trap 'rm -rf "$out_dir"' EXIT

# `|| true` on each send: a goal that ends ABORTED is a non-zero exit for the
# CLI, and here that is the expected outcome for both goals, not a failure.
docker_cargo_run --rm -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -v "$out_dir:/out" -w /repo/ros/cspace-ros "$IMAGE" bash -c '
  set -e
  "$CARGO_TARGET_DIR/debug/move_group" '"$URDF $SRDF"' 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  ros2 action list >/out/actions.txt 2>&1

  send() { # <name> <goal yaml>
    timeout 40 ros2 action send_goal --feedback /execute_trajectory \
      moveit_msgs/action/ExecuteTrajectory "$2" >"/out/$1.txt" 2>&1 || true
  }

  # An empty goal: no trajectory, no controller names. Upstream reads neither
  # field on the branch this port lands on, so this is the whole of what its
  # handler sees.
  send empty "{}"

  # The same endpoint with a goal a real backend would have something to do
  # with: two waypoints on the fixture joint and a named controller. The
  # assertions below require this to get byte-identical treatment.
  send loaded \
    "{trajectory: {joint_trajectory: {joint_names: [j1], points: [{positions: [0.0], time_from_start: {sec: 0}}, {positions: [0.5], time_from_start: {sec: 1}}]}}, controller_names: [fake_controller]}"

  cp /tmp/node.stderr /out/node.stderr
'

# Upstream's name for this action, verbatim and unqualified
# (capability_names.hpp:45). A rename reddens here first, with its own
# message, instead of surfacing as "the goal was never answered".
assert_line "action name" "/execute_trajectory" "$out_dir/actions.txt"

for goal in empty loaded; do
  assert_has "$goal goal acceptance" "Goal accepted with ID:" "$out_dir/$goal.txt"

  # The terminal transition, asserted in both directions. `assert_lacks
  # SUCCEEDED` is not redundant with `assert_has ABORTED`: the CLI prints one
  # status line, but a future handler that terminated a goal twice would print
  # both, and "the client was told it worked" is the outcome this endpoint
  # exists to never produce.
  assert_has "$goal terminal state" "Goal finished with status: ABORTED" "$out_dir/$goal.txt"
  assert_lacks "$goal terminal state" "Goal finished with status: SUCCEEDED" "$out_dir/$goal.txt"

  # CONTROL_FAILED, upstream's own code for this condition
  # (execute_trajectory_action_capability.cpp:93).
  assert_has "$goal code" "val: -4" "$out_dir/$goal.txt"
  # The three codes a working backend answers with, each excluded by name.
  # `val: 1` in particular is what the other possible port -- a server
  # reporting SUCCESS having executed nothing -- would send.
  assert_lacks "$goal code" "val: 1" "$out_dir/$goal.txt"
  assert_lacks "$goal code" "val: -7" "$out_dir/$goal.txt"
  assert_lacks "$goal code" "val: -6" "$out_dir/$goal.txt"

  assert_has "$goal source" "source: $SOURCE_STRING" "$out_dir/$goal.txt"
  assert_has "$goal message" "$BRANCH_MSG" "$out_dir/$goal.txt"

  # No feedback. Upstream publishes none on this branch, and a port that
  # published a MONITOR state would be telling the client execution started.
  assert_lacks "$goal feedback" "Feedback:" "$out_dir/$goal.txt"
done

# The uniformity claim, made mechanical. The module states that the goal's
# `trajectory` and `controller_names` are never inspected; if that ever stops
# being true, the two replies stop agreeing. Compared after stripping the
# goal echo and the per-run UUID, which differ between the two by
# construction.
reply() { # <file>
  sed -n '/^Result:/,$p' "$1"
}
if ! diff <(reply "$out_dir/empty.txt") <(reply "$out_dir/loaded.txt") >"$out_dir/reply.diff"; then
  echo "--- differing replies ---" >&2
  cat "$out_dir/reply.diff" >&2
  fail "an empty goal and a loaded one got different answers -- this endpoint's rule is
FAIL supposed to be uniform, because upstream's branch returns before reading either field"
fi

echo "OK execute_trajectory: /execute_trajectory accepted both goals over DDS and aborted"
echo "OK execute_trajectory: both with an identical CONTROL_FAILED (val=-4) reply naming"
echo "OK execute_trajectory: $SOURCE_STRING, publishing no feedback"
