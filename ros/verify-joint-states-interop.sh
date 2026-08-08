#!/bin/bash
# The `joint_states` leg of ros/verify-ros-interop.sh, which calls this as one
# of its steps.
#
# Two legs, because the topic and the client's use of it fail differently:
#
#   Leg A -- the wire. The topic exists, carries this model's joints, and --
#   the assertion this endpoint turns on -- its `header.stamp` MOVES. A
#   latched publisher that sent one message at startup would satisfy every
#   other check here and satisfy no client at all: `waitForCurrentState(t, w)`
#   loops while `current_state_time_ < t` with `t` the caller's own `now()`
#   (current_state_monitor.cpp:240), so a stamp fixed at startup is older than
#   every call ever made after it. Two consecutive samples with different
#   stamps is the smallest fact that rules that out.
#
#   Leg B -- the client. Upstream's unmodified `MoveGroupInterface` calls
#   `getCurrentState()` and gets back the value this gate pushed into the
#   node's scene beforehand. That closes the loop leg A cannot: leg A shows a
#   message with the right shape, not that upstream's `CurrentStateMonitor`
#   accepts it, and the two disagree for reasons -- a name that is a variable
#   rather than a joint, a joint the model calls multi-DOF, arrays of
#   different lengths -- that all look fine in `ros2 topic echo`.
#
# The distinctive value is what makes leg B an end-to-end measurement rather
# than a default-vs-default coincidence: 0.375 is pushed onto `/planning_scene`
# first, so a publisher sending zeros, or the model's own defaults, or a
# constant, reads differently from one relaying the node's monitored state.
#
# Leg B needs the oracle image and exits 3 without it, in the shape
# ros/verify-move-action-interop.sh established.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"
URDF=/repo/ros/fixtures/one_joint.urdf
SRDF=/repo/ros/fixtures/one_joint.srdf

# The value pushed through `/planning_scene` and expected back out of
# `getCurrentState()`. Inside j1's `[-1, 1]` limits (ros/fixtures/one_joint.urdf)
# and not a value any default produces.
CURRENT_J1=0.375

DOMAIN_ID="${ROS_DOMAIN_ID:-$((($$ % 100) + 1))}"
echo "=== joint_states (ROS_DOMAIN_ID=$DOMAIN_ID) ==="

fail() {
  echo "FAIL $*" >&2
  exit 1
}

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
# Leg A -- the wire
###############################################################################
out_dir="$(mktemp -d)"
trap 'rm -rf "$out_dir"' EXIT

docker_cargo_run --rm -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -v "$out_dir:/out" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  "$CARGO_TARGET_DIR/debug/move_group" '"$URDF $SRDF"' 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  ros2 topic list >/out/topics.txt 2>&1

  # Two samples, separately, so the stamps below come from two distinct
  # messages rather than from one message printed twice.
  timeout 20 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState >/out/first.txt 2>&1 || true
  timeout 20 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState >/out/second.txt 2>&1 || true

  # The clock the node stamps with, read from the same container at the same
  # time, so leg A can say the stamps are current rather than merely unequal.
  date +%s >/out/wall.txt

  cp /tmp/node.stderr /out/node.stderr
'

assert_line "topic name" "/joint_states" "$out_dir/topics.txt"

for sample in first second; do
  # The joint the fixture has, by joint name. `j1` is also its only variable
  # name, so this alone cannot tell a joint-name publisher from a
  # variable-name one -- leg B's model, which looks names up with
  # `getJointModel`, is what settles that.
  assert_has "$sample names" "- j1" "$out_dir/$sample.txt"
  # No velocity, no effort: upstream reads them only under `copy_dynamics_`,
  # and a zero-filled array would claim the robot is stationary rather than
  # that nothing measured it.
  assert_has "$sample velocity" "velocity: []" "$out_dir/$sample.txt"
  assert_has "$sample effort" "effort: []" "$out_dir/$sample.txt"
done

# An unset or wrong-base stamp is the failure this endpoint dies of quietly: it
# satisfies every shape check above and no `waitForCurrentState` call. It is
# checked below on the *parsed* seconds field rather than here as text, because
# text cannot say which field it read -- `grep -F 'sec: 0'` matches the
# `nanosec: 0` line as well, so as a check for an unset stamp it fires on a
# legitimate message whose nanosecond happens to land on zero and never
# separates the two fields it is named for. `stamp_of` anchors
# `^[[:space:]]*sec:`, which `nanosec:` cannot match.

stamp_of() { # <file> -> the sec field of the header stamp
  sed -n 's/^[[:space:]]*sec: \([0-9-]*\)$/\1/p' "$1" | head -1
}
nanosec_of() { # <file>
  sed -n 's/^[[:space:]]*nanosec: \([0-9]*\)$/\1/p' "$1" | head -1
}

first_stamp="$(stamp_of "$out_dir/first.txt").$(nanosec_of "$out_dir/first.txt")"
second_stamp="$(stamp_of "$out_dir/second.txt").$(nanosec_of "$out_dir/second.txt")"
echo "joint_states stamps: $first_stamp then $second_stamp"
if [ "$first_stamp" = "$second_stamp" ]; then
  cat "$out_dir/first.txt" >&2
  fail "two consecutive joint_states samples carry the same header.stamp ($first_stamp) --
FAIL a client's waitForCurrentState compares that stamp against its own now(), so a
FAIL stamp that never advances can satisfy no call made after the first publish"
fi

# Current, not merely moving: a stamp taken from a monotonic or zeroed clock
# advances too, and would still be older than every `node_->now()` a client
# compares it against. Whole seconds are enough -- this is a decade-scale
# discrimination, not a millisecond one. An unset stamp (`sec: 0`) is 55 years
# off and fails here, which is why there is no separate zero check.
#
# Both samples, not just the first: one sample on the wall clock and the next
# on some other base satisfies the inequality above -- they differ -- and would
# leave every call after it comparing two time bases.
wall="$(cat "$out_dir/wall.txt")"
for sample in first second; do
  sec="$(stamp_of "$out_dir/$sample.txt")"
  if [ -z "$sec" ]; then
    cat "$out_dir/$sample.txt" >&2
    fail "the $sample joint_states sample carries no header.stamp.sec field at all"
  fi
  if [ "$((wall - sec))" -gt 60 ] || [ "$((sec - wall))" -gt 60 ]; then
    fail "the $sample joint_states sample is stamped $sec but the container's wall clock
FAIL read $wall -- the stamp is not on the clock a client's node_->now() reads, so
FAIL waitForCurrentState would compare two different time bases"
  fi
done

echo "OK joint_states: the topic carries this model's joints with no velocity or"
echo "OK joint_states: effort, and two consecutive samples carry different, current stamps"

###############################################################################
# Leg B -- upstream's C++ MoveGroupInterface calling getCurrentState()
###############################################################################
echo "=== joint_states (leg B: upstream C++ getCurrentState) ==="

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
ORACLE_STAMP="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
ORACLE_IMAGE="${ORACLE_IMAGE:-$(oracle_image_tag "$ORACLE_STAMP")}"
PROBE_IMAGE="${PROBE_IMAGE:-moveit-rs/move-group-interface-probe:${ORACLE_STAMP:0:16}}"

if ! docker image inspect "$ORACLE_IMAGE" >/dev/null 2>&1; then
  echo "SKIP $ORACLE_IMAGE not built -- upstream's own CurrentStateMonitor never read"
  echo "SKIP this topic, so leg A above stands alone: it shows a message with the right"
  echo "SKIP shape, not that upstream accepts it as a complete state."
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

NET="moveit-rs-joint-states-$$"
NODE_CTR="moveit-rs-joint-states-node-$$"
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

# Push the distinctive value into the node's monitored scene first. A full
# scene (`is_diff: false`) rather than a diff, so the state that comes back is
# unambiguously this message's and not a default the node happened to hold.
docker_cargo_run --rm --network "$NET" -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" bash -c "
    timeout 20 ros2 topic pub --once --qos-reliability reliable /planning_scene \
      moveit_msgs/msg/PlanningScene \
      '{is_diff: false, robot_state: {is_diff: false, joint_state: {name: [j1], position: [$CURRENT_J1]}}}'
  " >/dev/null
sleep 1

timeout 180 docker run --rm --network "$NET" \
  -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
  -v "$REPO_ROOT:/repo" --entrypoint bash "$PROBE_IMAGE" -c "
    source /opt/ros/\$ROS_DISTRO/setup.bash
    source /ws/install/setup.bash
    ros2 run move_group_interface_probe move_group_interface_probe \
      $URDF $SRDF arm default-start current-state
  " >"$leg_b_out" 2>&1 || true

grep '^PROBE ' "$leg_b_out" || true

# `received`, not merely "no crash": `getCurrentState` returns a null pointer
# on timeout and the probe prints which happened. Asserted in both directions,
# because a probe that stopped printing the line at all would pass a bare
# `assert_lacks timeout`.
assert_has "leg B state" "PROBE current_state=received" "$leg_b_out"
assert_lacks "leg B state" "PROBE current_state=timeout" "$leg_b_out"

# The value this gate put into the node, arriving back through the node's
# joint_states publisher and upstream's own CurrentStateMonitor. This is the
# assertion that makes the leg end-to-end: every other check here passes
# against a publisher that sends a constant.
assert_has "leg B value" "PROBE current_state_variable name=j1 position=$CURRENT_J1" "$leg_b_out"

echo "OK joint_states leg B: upstream's unmodified MoveGroupInterface::getCurrentState()"
echo "OK joint_states leg B: returned the j1=$CURRENT_J1 this gate pushed into the node's"
echo "OK joint_states leg B: scene, so the state crossed /planning_scene, the node's"
echo "OK joint_states leg B: monitored scene, /joint_states and upstream's CurrentStateMonitor"
