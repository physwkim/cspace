#!/bin/bash
# The planner-parameter legs of ros/verify-ros-interop.sh, which calls this as
# one of its steps. Split out rather than inlined there for the reason
# ros/verify-move-action-interop.sh is: several panels are landing endpoints
# into ros/moveit-ros at once, and a per-capability gate file is what keeps
# their branches off each other -- the same split upstream makes between
# capabilities.
#
# The three services are one upstream capability
# (`query_planners_service_capability.cpp` creates all three in one
# `initialize()`), so they are gated in one file.
#
# What only a live graph can show, and what these legs are therefore for:
#
#   1. The three services exist under the names an unmodified
#      `MoveGroupInterface` resolves
#      (`move_group/capability_names.hpp:46`, `:48`, `:50`).
#      The unit tests drive the handlers directly and would keep passing if the
#      node advertised nothing at all.
#   2. `query_planner_interface`'s answer is derived from the linked
#      `PLANNER_MANAGERS` `distributed_slice` and actually reaches a client.
#      A `distributed_slice` that fails to link produces an empty list, which
#      is a valid message and reads exactly like "no planners configured".
#   3. **A `set` is not silently dropped.** `SetPlannerParams`'s response is
#      empty -- no field, no error code -- so a client cannot tell a stored
#      write from a discarded one. The only observation that separates them is
#      a `get` afterwards, over the same wire, and that is what leg B does.
#
# What these legs do NOT check: that a plan honours a stored configuration.
# Nothing here hands the store to a planner -- upstream's `setParams` ends at
# `setPlannerConfigurations` on the instance the pipeline plans with, and this
# port has no equivalent -- so there is no such behaviour to assert yet. These
# legs prove the store round trips over DDS and nothing beyond that. Stated
# about the wiring rather than about whether the node plans, so it does not
# quietly become false on the round that makes planning reachable.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"
URDF=/repo/ros/fixtures/one_joint.urdf
SRDF=/repo/ros/fixtures/one_joint.srdf

# Seven worktrees share this docker daemon and these legs match replies by
# content, not by sender -- the domain id keeps a concurrent copy of this gate
# off the same graph. Derived from `$$` so a failing run is reproducible with
# the id printed in its own output, the same rule the move_action legs use.
DOMAIN_ID="${ROS_DOMAIN_ID:-$((($$ % 100) + 1))}"
echo "planner_params legs: ROS_DOMAIN_ID=$DOMAIN_ID"

out="$(mktemp)"
trap 'rm -f "$out"' EXIT

fail() {
  echo "FAIL $*" >&2
  exit 1
}

# `if ! grep` rather than `grep || fail`: under `set -e` a `grep ... && fail`
# compound returns grep's 1 when the needle is absent and aborts the script at
# that line, so the assertion would report nothing at all. Same shape
# ros/verify-move-action-interop.sh records.
assert_has() { # <what> <fixed string> <file>
  if ! grep -qF -- "$2" "$3"; then
    echo "--- captured output ---" >&2
    cat "$3" >&2
    fail "$1: expected this string and did not find it: $2"
  fi
}

# `ros2 service call` prints the request back before the reply
# ("requester: making request: ..._Request(...)"), and for these services the
# echo carries the *same* key/value text as the reply it is asking about. An
# assertion over the whole transcript therefore cannot tell "the node stored
# this" from "ros2 printed my own request back" -- the first version of this
# file asserted over the whole file, and its `values=['9.9']` negative could
# never pass, because the rejected request's own echo satisfied it.
#
# So every reply assertion below is scoped to one call: the `_Response(` line
# inside the block that follows that call's `@@@` marker. A request echo is
# structurally out of reach, not merely unlikely.
reply_of() { # <marker>
  awk -v marker="@@@ $1" '
    index($0, marker) == 1 { inblock = 1; next }
    /^@@@ / { inblock = 0 }
    inblock && /_Response\(/ { print }
  ' "$out"
}

# The empty-reply guard is in BOTH helpers on purpose: without it a mistyped
# marker makes `assert_reply_lacks` pass over an empty string, which is a
# check that cannot fail.
assert_reply() { # <what> <marker> <fixed string>
  local reply
  reply="$(reply_of "$2")"
  if [[ -z $reply ]]; then
    echo "--- captured output ---" >&2
    cat "$out" >&2
    fail "$1: no reply captured for the '$2' call -- nothing was checked"
  fi
  if ! grep -qF -- "$3" <<<"$reply"; then
    echo "--- reply for '$2' ---" >&2
    printf '%s\n' "$reply" >&2
    fail "$1: expected this string in that reply and did not find it: $3"
  fi
}

assert_reply_lacks() { # <what> <marker> <fixed string>
  local reply
  reply="$(reply_of "$2")"
  if [[ -z $reply ]]; then
    echo "--- captured output ---" >&2
    cat "$out" >&2
    fail "$1: no reply captured for the '$2' call -- nothing was checked"
  fi
  if grep -qF -- "$3" <<<"$reply"; then
    echo "--- reply for '$2' ---" >&2
    printf '%s\n' "$reply" >&2
    fail "$1: this string must not appear in that reply and did: $3"
  fi
}

# Whole line, for `ros2 service list`'s one-name-per-line output. A substring
# match would accept `/get_planner_params_2` as `/get_planner_params`, and a
# renamed endpoint that still contains the old name as a prefix is exactly the
# regression this is for.
assert_line() { # <what> <exact line> <file>
  if ! grep -qxF -- "$2" "$3"; then
    echo "--- captured output ---" >&2
    cat "$3" >&2
    fail "$1: expected this exact line and did not find it: $2"
  fi
}

echo "=== planner_params (query / get / set over DDS) ==="

# One container, one node, all four calls in sequence: the set->get round trip
# only means anything against a single node process, since the store lives in
# that process.
docker run --rm -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  cargo build --bin move_group
  ./target/debug/move_group '"$URDF $SRDF"' 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  echo "@@@ service list"
  ros2 service list

  echo "@@@ query_planner_interface"
  timeout 20 ros2 service call /query_planner_interface \
    moveit_msgs/srv/QueryPlannerInterfaces "{}"

  echo "@@@ get before set (nothing stored yet)"
  timeout 20 ros2 service call /get_planner_params \
    moveit_msgs/srv/GetPlannerParams "{planner_config: RRTConnect}"

  echo "@@@ set (global)"
  timeout 20 ros2 service call /set_planner_params \
    moveit_msgs/srv/SetPlannerParams \
    "{planner_config: RRTConnect, params: {keys: [range], values: [\"0.1\"]}}"

  echo "@@@ get after set"
  timeout 20 ros2 service call /get_planner_params \
    moveit_msgs/srv/GetPlannerParams "{planner_config: RRTConnect}"

  echo "@@@ set for a pipeline this node does not serve"
  timeout 20 ros2 service call /set_planner_params \
    moveit_msgs/srv/SetPlannerParams \
    "{pipeline_id: ompl, planner_config: RRTConnect, params: {keys: [range], values: [\"9.9\"]}}"

  echo "@@@ get after the rejected set (must still read the stored 0.1)"
  timeout 20 ros2 service call /get_planner_params \
    moveit_msgs/srv/GetPlannerParams "{planner_config: RRTConnect}"

  echo "@@@ node stderr"
  cat /tmp/node.stderr
' >"$out" 2>&1

###############################################################################
# Leg A -- the three services exist, and the description comes off the registry
###############################################################################
assert_line "leg A query service name" "/query_planner_interface" "$out"
assert_line "leg A get service name" "/get_planner_params" "$out"
assert_line "leg A set service name" "/set_planner_params" "$out"

# Derived from `moveit_planners_sbp::registry::PLANNER_MANAGERS`, whose only
# registration today is `RRT_CONNECT` (`registry.rs`, `name: "rrt_connect"`).
# An empty `planner_interfaces` list -- what a `distributed_slice` that failed
# to link produces -- fails here rather than reading as "no planners".
# `QueryPlannerInterfaces`'s request is empty, so this is the one call whose
# echo could not have supplied the answer; it is scoped anyway, uniformly.
assert_reply "leg A description name" "query_planner_interface" "name='rrt_connect'"
# The advertised pipeline id is the one the other two services accept back.
assert_reply "leg A description pipeline_id" "query_planner_interface" "pipeline_id=''"
# Upstream's `PlannerManager::getPlanningAlgorithms` base default is
# `algs.clear()`, and no manager here overrides it.
assert_reply "leg A description planner_ids" "query_planner_interface" "planner_ids=[]"

echo "OK leg A: the three services are up and query_planner_interface answered"
echo "OK leg A: rrt_connect off the linked PLANNER_MANAGERS slice"

###############################################################################
# Leg B -- a set is stored, and a get sees it
###############################################################################
# The store starts empty, so the assertion after this one is this run's `set`
# landing and not a value that was there all along.
assert_reply "leg B empty before set" "get before set" "keys=[], values=[], descriptions=[]"

# The whole point of this leg. `SetPlannerParams` has an empty response, so a
# handler that parsed the request and threw it away is wire-indistinguishable
# from one that stored it -- until this `get`.
assert_reply "leg B stored params" "get after set" "keys=['range'], values=['0.1']"

# `descriptions` is inert on both legs upstream -- never filled by `getParams`,
# never read by `setParams`.
assert_reply "leg B descriptions stay empty" "get after set" "values=['0.1'], descriptions=[]"

# A pipeline this node does not serve is a miss (upstream's
# `resolvePlanningPipeline` over an empty pipeline map), so the rejected set
# must not have overwritten the stored value with 9.9 -- and the store must
# still hold what the accepted set put there.
assert_reply_lacks "leg B rejected set did not land" "get after the rejected set" "values=['9.9']"
assert_reply "leg B store survived the rejection" "get after the rejected set" "values=['0.1']"
assert_has "leg B rejection logged" "set_planner_params rejected: pipeline_id=\"ompl\"" "$out"

echo "OK leg B: a set crossed DDS, was stored, and a later get read it back;"
echo "OK leg B: a set for an unserved pipeline_id was rejected and logged"
echo "OK planner_params: both legs passed"
