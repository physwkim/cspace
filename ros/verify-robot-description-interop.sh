#!/bin/bash
# The `robot_description` / `robot_description_semantic` leg of
# ros/verify-ros-interop.sh, which calls this as one of its steps.
#
# Two legs, because two different things are in doubt and one run cannot
# settle both:
#
#   Leg A -- the wire. A LATE subscriber, connecting seconds after the node
#   published, receives both descriptions. That is the whole content of
#   "latched": a volatile publisher would leave a late subscriber with
#   nothing, and nothing is exactly what upstream's `waitForMessage` waits
#   ten seconds for before giving up (synchronized_string_parameter.cpp:82).
#   The subscriber's QoS is upstream's own, `transient_local` + `reliable`
#   (`:127`), for the same reason the payload strings are pinned: a leg that
#   negotiated its own compatible QoS would pass against a publisher upstream
#   could not read.
#
#   Leg B -- the client. Upstream's unmodified `MoveGroupInterface`,
#   constructed on a node with NO `robot_description` parameter at all, gets
#   its model from this node's topics. That is the requirement; leg A only
#   shows that bytes with the right shape are on the right topic. Leg B needs
#   the oracle image and exits 3 when it is absent, in the shape
#   ros/verify-move-action-interop.sh established for the same situation.
#
# What each assertion is for:
#
#   - both topics exist under upstream's exact names. The semantic one is
#     never written down upstream -- it is `ros_name + "_semantic"` built
#     inside the `RDFLoader` constructor (rdf_loader.cpp:96) -- so a port that
#     guessed `robot_semantic_description` would be invisible to every check
#     that only looked for `robot_description`;
#   - each topic carries the document it is named for, asserted in BOTH
#     directions. `revolute` appears only in the URDF and `tip_link` only in
#     the SRDF, so a port that swapped the two payloads fails here and
#     nowhere else: both topics would still be latched, both still carry
#     valid XML, and the client would build a model with no group;
#   - the client constructs with no parameter set, and its `plan()` still
#     comes back with a real trajectory. The plan is what proves the model it
#     built off the topic has the group it was asked for -- a constructor that
#     returned having loaded an empty SRDF would throw before this, and one
#     that loaded a group-less model would answer the plan differently.
#
# The strings are pinned to what the node publishes today, so the first change
# to either description has to come here, for the reason
# ros/verify-move-action-interop.sh states at greater length.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"
URDF=/repo/ros/fixtures/one_joint.urdf
SRDF=/repo/ros/fixtures/one_joint.srdf

# Seven worktrees share this docker daemon; derived from `$$` so a failing run
# is reproducible from the id in its own output. ros/verify-move-action-interop.sh
# records the full reasoning.
DOMAIN_ID="${ROS_DOMAIN_ID:-$((($$ % 100) + 1))}"
echo "=== robot_description (ROS_DOMAIN_ID=$DOMAIN_ID) ==="

fail() {
  echo "FAIL $*" >&2
  exit 1
}

# `if ! grep`, not `grep || fail`: under `set -e` a `grep ... && fail`
# compound aborts the script on grep's own 1 and the assertion reports
# nothing.
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

# Whole line, for `ros2 topic list`: a substring match accepts
# `/robot_description_semantic` as `/robot_description`, which is precisely
# the pair this leg has to keep apart.
assert_line() { # <what> <exact line> <file>
  if ! grep -qxF -- "$2" "$3"; then
    echo "--- captured output ---" >&2
    cat "$3" >&2
    fail "$1: expected this exact line and did not find it: $2"
  fi
}

docker_cargo_run --rm -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" \
  bash -c "cargo build --bin move_group" >&2

###############################################################################
# Leg A -- a late transient-local subscriber
###############################################################################
out_dir="$(mktemp -d)"
trap 'rm -rf "$out_dir"' EXIT

docker_cargo_run --rm -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -v "$out_dir:/out" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  "$CARGO_TARGET_DIR/debug/move_group" '"$URDF $SRDF"' 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT

  # Late on purpose: the publishes happened during startup, seconds before
  # this subscriber exists. Everything below is what `transient_local`
  # retains, and nothing else can be received on these topics -- the node
  # publishes each exactly once.
  sleep 3

  ros2 topic list >/out/topics.txt 2>&1

  # `--full-length`: without it `ros2 topic echo` prints the first ~150
  # characters and an ellipsis, which for these two topics is the fixture
  # files own header comment and nothing else -- every payload assertion below
  # would then be asserting about a comment.
  read_latched() { # <topic> <outfile>
    timeout 20 ros2 topic echo --once --full-length \
      --qos-durability transient_local --qos-reliability reliable --qos-depth 1 \
      "$1" std_msgs/msg/String >"$2" 2>&1 || true
  }
  read_latched /robot_description /out/urdf.txt
  read_latched /robot_description_semantic /out/srdf.txt

  cp /tmp/node.stderr /out/node.stderr
'

assert_line "topic name" "/robot_description" "$out_dir/topics.txt"
assert_line "semantic topic name" "/robot_description_semantic" "$out_dir/topics.txt"

# Each document on its own topic, and NOT on the other. The two needles are
# element tags rather than words, because both fixture files carry a prose
# header comment and `revolute` appears in the URDF's: a needle a comment can
# satisfy would pass against a swapped pair. `<axis` opens an element only the
# URDF has; `<group ` opens one only the SRDF has. Together they fail a swap,
# which every other assertion here would pass -- both topics would still be
# latched and both payloads still valid XML.
assert_has "urdf payload" "one_joint" "$out_dir/urdf.txt"
assert_has "urdf payload" "<axis" "$out_dir/urdf.txt"
assert_lacks "urdf payload" "<group " "$out_dir/urdf.txt"

assert_has "srdf payload" "one_joint" "$out_dir/srdf.txt"
assert_has "srdf payload" "<group " "$out_dir/srdf.txt"
assert_lacks "srdf payload" "<axis" "$out_dir/srdf.txt"

echo "OK robot_description: a late transient-local subscriber received both"
echo "OK robot_description: descriptions, each carrying its own document"

###############################################################################
# Leg B -- upstream's C++ MoveGroupInterface with no description parameter
###############################################################################
echo "=== robot_description (leg B: upstream C++ MoveGroupInterface, no parameter) ==="

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
ORACLE_STAMP="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
ORACLE_IMAGE="${ORACLE_IMAGE:-$(oracle_image_tag "$ORACLE_STAMP")}"
PROBE_IMAGE="${PROBE_IMAGE:-moveit-rs/move-group-interface-probe:${ORACLE_STAMP:0:16}}"

# Exit 3, not 0 and not 1, for the reason ros/verify-move-action-interop.sh
# spells out: leg A measured and leg B not run is a third outcome, and
# returning 0 for it would print a pass over a run in which upstream's own
# loader never read either topic.
if ! docker image inspect "$ORACLE_IMAGE" >/dev/null 2>&1; then
  echo "SKIP $ORACLE_IMAGE not built -- upstream's own RDFLoader never read these"
  echo "SKIP topics, so leg A above stands alone: it shows two latched strings with"
  echo "SKIP the right words in them, not that the client can build a model from them."
  echo "SKIP this is not a pass; build it with: sg docker -c tools/moveit-oracle/build.sh"
  exit 3
fi

CTX="$(mktemp -d "$REPO_ROOT/.probe-ctx.XXXXXX")"
trap 'rm -rf "$out_dir" "$CTX"' EXIT
cp -R "$REPO_ROOT/ros/move_group_interface_probe" "$CTX/move_group_interface_probe"
cp "$REPO_ROOT/ros/move_group_interface_probe/Dockerfile" "$CTX/Dockerfile"
docker build \
  --build-arg "ORACLE_IMAGE=$ORACLE_IMAGE" \
  -t "$PROBE_IMAGE" \
  -f "$CTX/Dockerfile" \
  "$CTX"

NET="moveit-rs-robot-description-$$"
NODE_CTR="moveit-rs-robot-description-node-$$"
leg_b_out="$(mktemp)"

teardown() {
  docker rm -f "$NODE_CTR" >/dev/null 2>&1 || true
  docker network rm "$NET" >/dev/null 2>&1 || true
  rm -rf "$out_dir" "$CTX"
  rm -f "$leg_b_out"
}
trap teardown EXIT

docker network create "$NET" >/dev/null

docker_cargo_run -d --rm --name "$NODE_CTR" --network "$NET" \
  -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" \
  "$DOCKER_CARGO_TARGET_MOUNT/debug/move_group" "$URDF" "$SRDF" >/dev/null
sleep 3

# `description-from-topic`: the probe sets no `robot_description` parameter at
# all, so `getMainParameter` returns false and upstream's own
# `SynchronizedStringParameter` goes to the graph -- twice, once per
# description. The URDF/SRDF paths are still passed because the probe's
# trajectory grading builds its own `PlanningScene` from them; the client
# under test never reads them.
#
# 180s rather than the 120s the /move_action leg uses: two 10s description
# timeouts are reachable here if either topic is missing, and a leg that hit
# the shell's own timeout would report a hang where the interesting outcome is
# `PROBE constructing` with no `PROBE constructed` after it.
timeout 180 docker run --rm --network "$NET" \
  -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" --entrypoint bash "$PROBE_IMAGE" -c "
    source /opt/ros/\$ROS_DISTRO/setup.bash
    source /ws/install/setup.bash
    ros2 run move_group_interface_probe move_group_interface_probe \
      $URDF $SRDF arm default-start description-from-topic
  " >"$leg_b_out" 2>&1 || true

# Printed on success too: a gate whose measurement is only visible when it
# fails leaves the operator no way to see what passing meant.
grep '^PROBE ' "$leg_b_out" || true

# The mode, asserted before anything else it implies. Without this the whole
# leg passes unchanged against a probe that quietly fell back to parameters,
# which is exactly what it used to do for a misspelled mode word.
assert_has "leg B mode" "PROBE description=topic" "$leg_b_out"
assert_has "leg B constructed" "PROBE constructed" "$leg_b_out"

# The model came from the topics AND has the group: `plan()` reaching this
# node and coming back SUCCESS with a non-empty trajectory cannot happen
# through a group-less model. `source` is what says the answer crossed DDS
# rather than being built in the client (`move_group_interface.cpp:659-663`).
assert_has "leg B round trip" "PROBE plan val=1 source='moveit-ros/move_action'" "$leg_b_out"
assert_lacks "leg B client-local failure" "PROBE plan val=99999 source=''" "$leg_b_out"
assert_lacks "leg B empty trajectory" "PROBE points=0 " "$leg_b_out"

echo "OK robot_description leg B: upstream's unmodified MoveGroupInterface built its"
echo "OK robot_description leg B: model from this node's two latched topics with no"
echo "OK robot_description leg B: robot_description parameter set, and planned through it"
