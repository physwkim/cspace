#!/bin/bash
# Builds ros/moveit-ros and runs fmt/clippy/test inside the ROS 2 Rolling +
# Rust image (ros/Dockerfile). Named `verify-*`, not `check-*`, on purpose:
# it needs docker, and tools/ci/'s `check-*.sh` glob is run by CI runners
# that don't have it (PORTING-PLAN.md §129.4).
#
# Run by `tools/ci/verify-ros-interop.sh`, a thin caller that puts this
# script under `tools/ci/verify-all.sh`'s glob -- this script itself must
# stay outside `tools/ci/` (D5: ros/moveit-ros is its own `[workspace]`,
# and this needs docker, which `check-*.sh`'s CI runners do not have), but
# round 17's audit found it with no caller anywhere, which is the same
# never-runs shape as a gate no glob reaches, one level up.
#
# What this checks (all inside ros/Dockerfile's image, against
# ros/moveit-ros's own `[workspace]`, not the root workspace):
#   - `cargo fmt --check`
#   - `cargo clippy --all-targets -- -D warnings`
#   - `cargo test` (unit tests + doctests for ros/moveit-ros; nextest is not
#     installed in this image -- see the `run "test"` line below), with its
#     reported pass count checked against the `#[test]` count in
#     ros/moveit-ros/src -- a `cargo test` that silently compiles fewer
#     tests than the source contains (a stray `#[cfg]`, a filter typo) still
#     exits 0, and 0 passing is the same exit code as 0 tests existing to
#     run. See the `expected_tests`/`unit_summary` block below.
#   - `cargo doc --no-deps` (PORTING-PLAN.md §178: neither clippy
#     `--all-targets` nor `cargo test --doc` reaches rustdoc's own lints --
#     an unresolved intra-doc link or a link to a private item compiles and
#     tests clean but fails `cargo doc`, which is exactly how main went red
#     under the round-32 merge gate before this line existed)
#   - A live `/plan_kinematic_path` round trip over DDS (`run "live"` below,
#     PORTING-PLAN.md §241)
#   - Two live planner-parameter legs over DDS -- the three services exist,
#     `query_planner_interface`'s answer comes off the linked
#     `PLANNER_MANAGERS` slice, and a `set` is stored rather than silently
#     dropped (ros/verify-planner-params-interop.sh, PORTING-PLAN.md §274)
#   - A live `/execute_trajectory` leg over DDS
#     (ros/verify-execute-trajectory-interop.sh, called below)
#   - Two live `robot_description` / `robot_description_semantic` legs
#     (ros/verify-robot-description-interop.sh): a late transient-local
#     subscriber, and upstream's own `MoveGroupInterface` built on a node with
#     no description parameter at all, so its `RDFLoader` has nowhere to get a
#     model but these topics
#   - Two live `joint_states` legs (ros/verify-joint-states-interop.sh): the
#     stamp advances on the wall clock, and upstream's own
#     `CurrentStateMonitor` hands `getCurrentState()` back a distinctive value
#     this gate pushed into the node beforehand
#   - Two live `/move_action` legs over DDS, one of them driven by upstream's
#     own unmodified C++ `MoveGroupInterface` (ros/verify-move-action-interop.sh,
#     called at the end of this script). PORTING-PLAN.md §250 measured that
#     round trip once, by hand; those legs are what re-run it.
#
# What this does NOT check (read this list before wiring `ros/` into CI):
#   - No trajectory is compared against anything. Both live endpoints now
#     plan for real -- D8 landed `moveit_planners_sbp::registry::
#     RrtConnectManager` against `moveit_planning`'s own types -- and the live
#     legs assert that a plannable request comes back SUCCESS with a
#     non-empty trajectory. What they do not do is check that trajectory
#     against upstream: no oracle runs here, so "a plan came back" is the
#     claim, not "the same plan moveit2 would produce".
#   - Nothing here grades the trajectory upstream's own C++ client receives
#     for collision-freeness: leg B of ros/verify-move-action-interop.sh
#     prints the colliding count and asserts nothing about it, because
#     `one_joint.urdf` carries no collision geometry for a trajectory over it
#     to collide with. That the client gets a trajectory at all *is* checked
#     now, in both start-state spellings -- §250.6's rejection is gone and
#     §273 moved the §5 Phase 9 row to MET, so an earlier version of this
#     line calling that row UNMET was reporting a tree this script no longer
#     runs against.
#   - No in-process message round trip: every test in
#     ros/moveit-ros/src/**/*.rs constructs `r2r`-generated message structs
#     and converts them without ever crossing the middleware. The live legs
#     cover wire-format compatibility for the two endpoints they call and for
#     nothing else -- no topic, and no other service or action, is exercised.
#   - No cross-workspace check against `crates/`: ros/moveit-ros is its own
#     `[workspace]` (D5) built with its own `cargo` invocations here; this
#     script never builds or tests the root workspace, and the root
#     workspace's CI never builds this crate. A breaking change to a
#     `crates/moveit-*` public API is only caught here if this crate happens
#     to use the changed symbol and someone remembers to run this script.
#   - No moveit_msgs schema-drift check against upstream: this image builds
#     moveit_msgs from the pinned `third_party/moveit_msgs` checkout
#     (ros/Dockerfile); if that checkout is ever repinned to a newer
#     upstream tag, this script will catch a resulting compile break but not
#     a silent field-meaning change that still compiles.
#   - No oracle/fixture comparison: this is unrelated to
#     `tools/moveit-oracle`'s replay gate: it never runs the C++ oracle,
#     never touches fixtures, and its pass/fail carries no information about
#     numerical parity with moveit2.
#   - No performance/benchmark signal: gate is compile + unit-test pass/fail
#     only.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"

# `CollisionObject.operation` is declared `byte`
# (`moveit_msgs/msg/CollisionObject.msg:51`, with `byte ADD=0` at `:38` and
# `byte REMOVE=1` at `:41`), and rclpy models a ROS `byte` as a one-element
# `bytes` -- not an `int`. Writing `operation: 1` in a `ros2 topic pub` YAML
# therefore leaves the field at its default 0 and *still prints
# `publishing #1`*: a REMOVE published that way arrives as an ADD, and the
# publisher reports success. `!!binary` is the only spelling measured to
# reach the wire (`ros2 topic echo` shows `operation: "\x01"` for
# `!!binary AQ==`, and `operation: "\0"` for a bare `1`; the quoted-escape
# form `"\x01"` is rejected outright). See PORTING-PLAN.md §276.
#
# These two are the file's only definition of an operation number, exported
# into every container below, so no publish site writes one itself.
export OP_ADD="!!binary AA=="
export OP_REMOVE="!!binary AQ=="

run() {  # <label> <command...>
  local label="$1"
  shift
  echo "=== $label ==="
  docker run --rm -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" "$@"
}

run "fmt" bash -c "cargo fmt --check"
run "clippy" bash -c "cargo clippy --all-targets -- -D warnings"

# nextest is not installed in this image (round 1 scope); `cargo test` is
# the brief's stated fallback (PORTING-PLAN.md's Phase 9 round-1 brief).
#
# Real attributes are matched on their own line (rustfmt's own layout for
# them) to exclude the module's own prose and string literals that mention
# `#[test]` as text -- ros/moveit-ros/src/conversion_coverage.rs, which
# scans `#[test]` functions by name, has five such non-attribute mentions
# that a plain `rg -c '#\[test\]'` counts as real.
expected_tests="$(rg -c '^\s*#\[test\]\s*$' "$REPO_ROOT/ros/moveit-ros/src" -t rust |
  awk -F: '{s+=$2} END{print s+0}')"

echo "=== test ==="
# `|| test_status=$?` rather than a bare assignment followed by `$?`: under
# this script's own `set -e`, a failing command substitution aborts *at the
# assignment*, so the next line never runs, `$test_status` is never anything
# but 0, and the handler below is dead code. That costs the operator the
# whole diagnosis -- `$test_output` holds cargo's compiler errors and is
# printed on the line after the one that aborts, so a failing `cargo test`
# here produced no output at all beyond this `=== test ===` header.
test_status=0
test_output="$(docker run --rm -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" bash -c "cargo test" 2>&1)" ||
  test_status=$?
printf '%s\n' "$test_output"
if [[ $test_status -ne 0 ]]; then
  echo "FAIL cargo test failed" >&2
  exit 1
fi

# Every unit-test phase's own summary line, summed -- doctests print a
# separate, later "test result:" line under its own "   Doc-tests" header,
# excluded by the `sed` range below. Before PORTING-PLAN.md §241 added this
# crate's first `[[bin]]` target, exactly one unit-test binary
# existed (the lib), so "the last 'test result:' line before Doc-tests" and
# "the lib's own count" were the same line -- a `tail -1` here was enough.
# A `[[bin]]` target gives `cargo test` a second unit-test suite (its own
# "Running unittests src/bin/...", its own "test result:" line, 0 tests by
# this binary's own design) sandwiched between the lib's line and
# "   Doc-tests" -- `tail -1` silently started reading the bin's empty
# result instead of the lib's 174 (caught by this script itself, run
# against its own §241 change, before this fix landed). Summing every
# unit-test line in range is the general rule for however many `[[bin]]`
# targets this crate ends up with, not just the one that broke `tail -1`.
#
# `|| true` on both assignments below: under this script's own `set -e` and
# `pipefail`, `grep`'s legitimate "no match" exit (1) propagates to the whole
# pipeline and aborts the assignment before the `-z` check that follows it
# ever runs -- the same `test_status=$?` shape 48ef7ce closed, reappearing
# through pipefail instead of `$?`. Round 18's sweep found it: a unit-test run
# filtered down to nothing produced no diagnostic at all, not even this
# file's own "could not find..." message below, because the script had
# already died one line above it.
unit_lines="$(sed -n '1,/^   Doc-tests/p' <<<"$test_output" | grep -E '^test result: ' || true)"
if [[ -z "$unit_lines" ]]; then
  echo "FAIL could not find any unit-test 'test result:' line in cargo test's output -- nothing was checked." >&2
  exit 1
fi
actual_tests=0
while IFS= read -r unit_line; do
  n="$(grep -oE '[0-9]+ passed' <<<"$unit_line" | grep -oE '^[0-9]+' || true)"
  if [[ -z "$n" ]]; then
    echo "FAIL could not parse a passing-test count out of: $unit_line" >&2
    exit 1
  fi
  actual_tests=$((actual_tests + n))
done <<<"$unit_lines"
if [[ "$actual_tests" -ne "$expected_tests" ]]; then
  echo "FAIL cargo test reported $actual_tests passing unit test(s) across all unit-test binaries but ros/moveit-ros/src has $expected_tests '#[test]' function(s)." >&2
  echo "FAIL a stray #[cfg], a filter, or a renamed module silently dropped $((expected_tests - actual_tests)) of them from the run." >&2
  exit 1
fi
echo "OK $actual_tests/$expected_tests source-declared unit tests actually ran"

run "doc" bash -c "cargo doc --no-deps"

# Live round-trip (PORTING-PLAN.md §241): every check above compiles and
# unit-tests moveit-ros in-process -- this script's own "what this does NOT
# check" list at the top says so, and until this round it was true. This
# step is the one exception: it starts the real `move_group`
# binary against a fixture URDF/SRDF, sends it a real
# `moveit_msgs/srv/GetMotionPlan` request over live DDS with `ros2 service
# call` (not an in-process struct construction), and asserts the response
# carries the exact typed error this round's handler returns -- not "service
# not found", not a hang, not a wrong error code. `set -e` inside the
# `bash -c` string means any of the three checks failing propagates as this
# `docker run`'s own exit status, which this script's own `set -e` then
# aborts on -- no pipe sits between here and that exit code.
#
# The fixture moved out of a heredoc here and into ros/fixtures/ when the
# `/move_action` legs arrived: those run the robot through a second container
# built from a different image, and a client that loads a different robot than
# the node disagrees about group names for reasons that have nothing to do
# with what is being measured.
# A quoted heredoc, not an inline single-quoted argument: the needles below
# contain the apostrophes rustc and rosidl print around names
# (`planner 'rrt_connect' failed`, `group_name='arm'`), and every one of them
# would have closed a single-quoted `bash -c` word.
LIVE_SCRIPT=$(cat <<'EOS'
set -e
cargo build --bin move_group
./target/debug/move_group \
  /repo/ros/fixtures/one_joint.urdf /repo/ros/fixtures/one_joint.srdf &
server_pid=$!
trap "kill $server_pid 2>/dev/null || true" EXIT
sleep 3

# Boundary 1: a request naming a group and a goal. This is what D8 bought --
# before it there was no planner to hand the converted request to -- and it is
# asserted by something only a real plan can produce. `group_name` is derived
# from the returned trajectory and only when that trajectory is non-empty
# (`planning_response.cpp:48`, inside upstream's own
# `if (trajectory && !trajectory->empty())`), so it cannot appear on an empty
# answer the way a bare SUCCESS code could.
out="$(timeout 30 ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan \
  "{motion_plan_request: {group_name: arm, goal_constraints: [{joint_constraints: [{joint_name: j1, position: 0.5, tolerance_above: 0.001, tolerance_below: 0.001, weight: 1.0}]}]}}")"
echo "$out"
echo "$out" | grep -qF "val=1," || {
  echo "FAIL live round-trip: a plannable request did not come back SUCCESS (val=1)" >&2
  exit 1
}
echo "$out" | grep -qF "group_name='arm'" || {
  echo "FAIL live round-trip: SUCCESS carried no trajectory (group_name is empty)" >&2
  exit 1
}
# On the success reply too, not only the failure one below: `source` names the
# endpoint that built the answer, and a rule that held on one arm and not the
# other would leave the reply that matters most -- the one carrying a plan --
# unattributable. `src/bin/move_group.rs`'s `plan` is where the stamp happens,
# on both arms, so that this and the assertion below check the same thing.
echo "$out" | grep -qE "source=.moveit-ros/plan_kinematic_path\b" || {
  echo "FAIL live round-trip: the SUCCESS reply did not name the endpoint that built it" >&2
  exit 1
}

# Boundary 2: same endpoint, a request naming no group at all. It converts,
# reaches the planner, and fails *there* -- a different path from the
# conversion rejecting a message before any planner runs, and the reason the
# message has to name the planner rather than just carry a code.
out="$(timeout 15 ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "{}")"
echo "$out"
# `-E` with a trailing non-digit guard rather than a plain substring: §255's
# `grep -q "val=-1"` also matched `val=-16`, the other code this same handler
# returns (INVALID_GOAL_CONSTRAINTS, when the request does not convert) -- so
# it could not tell the two apart. The code changed; the guard is kept because
# the reason for it did not.
echo "$out" | grep -qE "val=99999([^0-9]|$)" || {
  echo "FAIL live round-trip: response did not carry the expected FAILURE (val=99999) error code" >&2
  echo "FAIL upstream answers both a null resolvePlanningPipeline and a pipeline that ran and" >&2
  echo "FAIL did not solve with FAILURE, in both capabilities (plan_service_capability.cpp:82-85," >&2
  echo "FAIL :92-97; move_action_capability.cpp:207-211,218-227)." >&2
  exit 1
}
echo "$out" | grep -qF "planner 'rrt_connect' failed: unknown joint model group" || {
  echo "FAIL live round-trip: response did not name the planner that failed" >&2
  exit 1
}
# `source` is what separates an answer built by this node from one built
# anywhere else -- the same assertion the move_action legs make about their
# own endpoint. It names the endpoint, not the binary: this reply used to
# carry the binary name and went stale on the wire the moment the binary was
# renamed (PORTING-PLAN.md §255), with nothing looking at it.
#
# `\b` and not a trailing `.`: without a right-hand word boundary this would
# match the `source=...plan_kinematic_path_server` it replaced just as well,
# and a check that accepts both spellings cannot tell them apart.
echo "$out" | grep -qE "source=.moveit-ros/plan_kinematic_path\b" || {
  echo "FAIL live round-trip: response did not carry source=moveit-ros/plan_kinematic_path" >&2
  exit 1
}
EOS
)
run "live" bash -c "$LIVE_SCRIPT"
echo "OK live round-trip: /plan_kinematic_path planned a real MotionPlanRequest over DDS and"
echo "OK live round-trip: reported the unplannable one as FAILURE naming rrt_connect"

# Leg C -- the planning-scene subscription (PORTING-PLAN.md §257). Same shape
# as the `/move_action` legs: publish on live DDS and assert the node answers
# *differently* afterwards. A leg that only published would pass against a
# subscription that dropped every message on the floor, so what is asserted
# here is not the publish but `/check_state_validity`'s verdict flipping
# either way across it:
#
#   step 1  empty world                 -> valid=True
#   step 2  FULL scene adding `blocker` -> valid=False, contacting `blocker`
#   step 3  DIFF adding `bystander`     -> valid=False, still contacting `blocker`
#   step 4  FULL with `bystander` only  -> valid=True
#
# Steps 3 and 4 are the whole full-vs-diff split, in both directions: a diff
# handled as a full scene clears `blocker` and turns step 3 True, and a full
# scene handled as a diff keeps `blocker` and turns step 4 False. Neither
# mutation can pass by fixing the other.
#
# Its own ROS_DOMAIN_ID, for the reason ros/verify-move-action-interop.sh
# records at greater length: seven worktrees share this docker daemon, and
# this leg's assertions read a scene that any concurrently-running copy could
# publish into. Derived from `$$` so a failing run is reproducible from the id
# printed in its own output. The `run` helper above deliberately stays
# domain-less, so this leg calls docker directly.
#
# The robot is written inline rather than added to ros/fixtures/one_joint.urdf
# because that file has no `<collision>` element at all -- with no robot
# geometry `/check_state_validity` answers True against any world, and this leg
# could not discriminate anything. Giving the shared fixture collision geometry
# is the better home for it (one robot for all three legs, which is what that
# file's own comment is protecting), but `ros/fixtures/` is outside this
# round's fence; unlike the `/move_action` legs this one runs in a single
# container and shares its robot with nobody, so an inline copy costs no
# cross-container agreement here.
SCENE_DOMAIN_ID="${ROS_DOMAIN_ID:-$((($$ % 100) + 1))}"
echo "=== scene-topic (ROS_DOMAIN_ID=$SCENE_DOMAIN_ID) ==="
docker run --rm -e "ROS_DOMAIN_ID=$SCENE_DOMAIN_ID" \
  -e OP_ADD -e OP_REMOVE \
  -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  cat >/tmp/boxed.urdf <<\URDF
<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </collision>
  </link>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>
URDF
  cat >/tmp/boxed.srdf <<\SRDF
<?xml version="1.0"?>
<robot name="one_joint">
  <group name="arm"><chain base_link="base_link" tip_link="tip"/></group>
</robot>
SRDF

  cargo build --bin move_group
  ./target/debug/move_group /tmp/boxed.urdf /tmp/boxed.srdf 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  fail() {
    echo "FAIL scene-topic: $*" >&2
    echo "--- last /check_state_validity reply ---" >&2
    cat /tmp/validity >&2 2>/dev/null || true
    echo "--- node stderr ---" >&2
    cat /tmp/node.stderr >&2
    exit 1
  }

  # Whole line, not a substring: ros2 topic list prints one name per line, and
  # /planning_scene_2 contains /planning_scene -- the same rename hazard the
  # assert_line helper in ros/verify-move-action-interop.sh exists for.
  ros2 topic list >/tmp/topics
  grep -qxF /planning_scene /tmp/topics || {
    cat /tmp/topics >&2
    fail "the node is not subscribed to /planning_scene"
  }

  validity() {  # <step label>
    out="$(timeout 20 ros2 service call /check_state_validity \
      moveit_msgs/srv/GetStateValidity "{}")" ||
      fail "$1: /check_state_validity did not answer"
    # The reply only: ros2 service call echoes the request too, and only one
    # of the two carries valid=.
    printf %s "$out" | sed -n "/^response:/,\$p" >/tmp/validity
    echo "PROBE $1: $(grep -o "valid=[A-Za-z]*" /tmp/validity) $(grep -o "contact_body_2=.[a-z]*." /tmp/validity | tr "\n" " ")"
  }

  publish() {  # <step label> <is_diff> <object id> <x>
    timeout 25 ros2 topic pub -1 -w 1 /planning_scene moveit_msgs/msg/PlanningScene \
      "{is_diff: $2, world: {collision_objects: [{header: {frame_id: base_link}, id: $3, operation: $OP_ADD, pose: {position: {x: $4}, orientation: {w: 1.0}}, primitives: [{type: 1, dimensions: [0.2, 0.2, 0.2]}], primitive_poses: [{orientation: {w: 1.0}}]}]}}" \
      >/dev/null || fail "$1: publishing on /planning_scene failed"
    sleep 2
  }

  validity "step 1 (empty world)"
  grep -q "valid=True" /tmp/validity ||
    fail "step 1: an empty world must be valid, or nothing the later steps assert means anything"

  publish "step 2" false blocker 0.0
  validity "step 2 (full scene, blocker at the robot)"
  grep -q "valid=False" /tmp/validity ||
    fail "step 2: the published scene never reached the node -- a box on top of the robot is a collision"
  grep -q "contact_body_2=.blocker." /tmp/validity ||
    fail "step 2: a collision was reported, but not against the published object"

  publish "step 3" true bystander 10.0
  validity "step 3 (diff, bystander far away)"
  grep -q "valid=False" /tmp/validity ||
    fail "step 3: a diff must not clear the world, but blocker is gone -- the diff was applied as a full scene"
  grep -q "contact_body_2=.blocker." /tmp/validity ||
    fail "step 3: blocker survived the diff but is no longer the contacting body"

  publish "step 4" false bystander 10.0
  validity "step 4 (full scene, bystander only)"
  grep -q "valid=True" /tmp/validity ||
    fail "step 4: a full scene must clear the world, but blocker survived -- the full scene was applied as a diff"

  echo "--- node stderr ---"
  cat /tmp/node.stderr
'
echo "OK scene-topic: /planning_scene reached the node over DDS, and /check_state_validity"
echo "OK scene-topic: answered True/False/False/True across an empty world, a full scene, a diff, and a full scene"

# The planner-parameter trio, in its own file: they are one upstream
# capability (`query_planners_service_capability.cpp` creates all three in one
# `initialize()`), and a per-capability gate file is what keeps the panels
# landing endpoints in parallel off each other.
"$REPO_ROOT/ros/verify-planner-params-interop.sh"

# Leg D -- the two inbound topics a MoveGroupInterface client publishes to
# (PORTING-PLAN.md §276): `attached_collision_object` (attachObject/
# detachObject) and `trajectory_execution_event` (stop). Both are
# SUBSCRIBERS on this node, so "bound" means a subscription that actually
# receives -- doc/client-endpoint-surface.md's instrument reads the role from
# the r2r factory and calls a name opened the other way `role-mismatch`, not
# bound. This leg checks the half that instrument cannot: that the received
# message changes what the node answers.
#
# For the attach that means a collision query flipping on an *attached body*
# rather than on a world object, which is a different code path
# (PlanningScene::check_collision folds attached bodies into the robot
# object; a world object goes in through the world). The world holds
# `bystander` at x=10 throughout and the robot sits at the origin, so the
# only thing that can put geometry at x=10 on the robot side is the attach.
#
# For the stop event the observable is the node's own stderr, and that is a
# weaker observable than a flipped answer -- said plainly rather than dressed
# up. Nothing in this workspace executes a trajectory (no
# moveit_controller_manager is ported), so a `stop` has no execution to
# preempt and upstream's own no-op arm is the reachable one. What the two
# assertions below do discriminate is that the payload is *decoded*: a
# callback that ignored its payload could not produce both the no-op line and
# the unknown-event line naming `wobble`.
SCENE_DOMAIN_ID="${ROS_DOMAIN_ID:-$((($$ % 100) + 2))}"
echo "=== inbound-topics (ROS_DOMAIN_ID=$SCENE_DOMAIN_ID) ==="
docker run --rm -e "ROS_DOMAIN_ID=$SCENE_DOMAIN_ID" \
  -e OP_ADD -e OP_REMOVE \
  -v "$REPO_ROOT:/repo" -w /repo/ros/moveit-ros "$IMAGE" bash -c '
  set -e
  cat >/tmp/boxed.urdf <<\URDF
<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </collision>
  </link>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>
URDF
  cat >/tmp/boxed.srdf <<\SRDF
<?xml version="1.0"?>
<robot name="one_joint">
  <group name="arm"><chain base_link="base_link" tip_link="tip"/></group>
</robot>
SRDF

  cargo build --bin move_group
  ./target/debug/move_group /tmp/boxed.urdf /tmp/boxed.srdf 2>/tmp/node.stderr &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3

  fail() {
    echo "FAIL inbound-topics: $*" >&2
    echo "--- last /check_state_validity reply ---" >&2
    cat /tmp/validity >&2 2>/dev/null || true
    echo "--- node stderr ---" >&2
    cat /tmp/node.stderr >&2
    exit 1
  }

  ros2 topic list >/tmp/topics
  for t in /attached_collision_object /trajectory_execution_event; do
    grep -qxF "$t" /tmp/topics || {
      cat /tmp/topics >&2
      fail "the node is not subscribed to $t"
    }
  done

  validity() {  # <step label>
    out="$(timeout 20 ros2 service call /check_state_validity \
      moveit_msgs/srv/GetStateValidity "{}")" ||
      fail "$1: /check_state_validity did not answer"
    printf %s "$out" | sed -n "/^response:/,\$p" >/tmp/validity
    echo "PROBE $1: $(grep -o "valid=[A-Za-z]*" /tmp/validity) $(grep -o "contact_body_[12]=.[a-z]*." /tmp/validity | tr "\n" " ")"
  }

  # A world object far from the robot. Everything below turns on whether the
  # robot side reaches out to it.
  timeout 25 ros2 topic pub -1 -w 1 /planning_scene moveit_msgs/msg/PlanningScene \
    "{is_diff: false, world: {collision_objects: [{header: {frame_id: base_link}, id: bystander, operation: $OP_ADD, pose: {position: {x: 10.0}, orientation: {w: 1.0}}, primitives: [{type: 1, dimensions: [0.2, 0.2, 0.2]}], primitive_poses: [{orientation: {w: 1.0}}]}]}}" \
    >/dev/null || fail "seeding the world with bystander failed"
  sleep 2
  validity "step 1 (world object far away, nothing attached)"
  grep -q "valid=True" /tmp/validity ||
    fail "step 1: a world object 10m away must not collide, or nothing below means anything"

  # <operation> is `$OP_ADD`/`$OP_REMOVE` -- never a number. See the top of
  # this file for why a bare int here publishes ADD no matter what it says.
  attach() {  # <step label> <operation> <x>
    timeout 25 ros2 topic pub -1 -w 1 /attached_collision_object \
      moveit_msgs/msg/AttachedCollisionObject \
      "{link_name: base_link, object: {header: {frame_id: base_link}, id: held, operation: $2, pose: {position: {x: $3}, orientation: {w: 1.0}}, primitives: [{type: 1, dimensions: [0.2, 0.2, 0.2]}], primitive_poses: [{orientation: {w: 1.0}}]}}" \
      >/dev/null || fail "$1: publishing on /attached_collision_object failed"
    sleep 2
  }

  attach "step 2" "$OP_ADD" 10.0
  validity "step 2 (a body attached to base_link, reaching bystander)"
  grep -q "valid=False" /tmp/validity ||
    fail "step 2: the attached body never reached the scene -- attaching geometry onto bystander is a collision"
  grep -q "held" /tmp/validity ||
    fail "step 2: a collision was reported, but not against the attached body"
  # body_type 2 is RobotAttached. A world object would report 1, so this is
  # what separates "the attach was applied as an attached body" from "the
  # attach was applied as a world object", which would collide too.
  grep -qE "body_type_[12]=2" /tmp/validity ||
    fail "step 2: the contact names held but not as an attached body (body_type 2) -- the attach landed in the world instead"

  attach "step 3" "$OP_REMOVE" 10.0
  validity "step 3 (the same body detached)"
  grep -q "valid=True" /tmp/validity ||
    fail "step 3: REMOVE on attached_collision_object did not detach -- the body is still on the robot"

  event() {  # <step label> <payload>
    timeout 25 ros2 topic pub -1 -w 1 /trajectory_execution_event \
      std_msgs/msg/String "{data: $2}" \
      >/dev/null || fail "$1: publishing on /trajectory_execution_event failed"
    sleep 2
  }

  event "step 4" stop
  grep -q "trajectory_execution_event stop: nothing to stop" /tmp/node.stderr ||
    fail "step 4: a stop with nothing executing must take the defined no-op arm"
  if grep -q "preempted execution" /tmp/node.stderr; then
    fail "step 4: the node claims it preempted an execution, but nothing in this workspace executes"
  fi
  echo "PROBE step 4 (stop while idle): no-op arm taken, nothing claimed preempted"

  event "step 5" wobble
  grep -q "unknown trajectory_execution_event type: .wobble." /tmp/node.stderr ||
    fail "step 5: an unrecognised event must be reported by name, not silently dropped"
  echo "PROBE step 5 (unknown event): reported by name"

  echo "--- node stderr ---"
  cat /tmp/node.stderr
'
echo "OK inbound-topics: /attached_collision_object flipped a collision answer via an attached"
echo "OK inbound-topics: body (body_type 2) and back, and /trajectory_execution_event decoded both payloads"

# `/execute_trajectory`, in its own file for the same reason the scene-topic
# leg above takes its own domain id: it starts its own node and asserts on
# replies matched by content. Called bare rather than captured, unlike the
# `/move_action` legs below -- it has no skip outcome to report, so `set -e`
# aborting on its exit status is the whole handling it needs.
"$REPO_ROOT/ros/verify-execute-trajectory-interop.sh"

# The sub-gates that orchestrate their own containers and a docker network, and
# whose leg B runs upstream's own unmodified C++ client. Each of them exits 3
# when the oracle image is absent, which is a third outcome and not a pass:
# their leg A still measured, their leg B never ran.
#
# Called through this helper rather than bare, for two reasons. Under `set -e` a
# bare call aborts this script on the 3, and the summary below -- the one place
# a skip is reported -- never runs. And the summary has to name *which* legs did
# not run: with one status variable and one `case` arm per gate, that naming was
# restated per gate and silently omitted for the next one added.
skipped=()
run_oracle_gate() { # <script relative to REPO_ROOT> <name for the summary>
  local status=0
  "$REPO_ROOT/$1" || status=$?
  case "$status" in
    0) ;;
    3) skipped+=("$2") ;;
    *) exit "$status" ;;
  esac
}

run_oracle_gate ros/verify-robot-description-interop.sh "robot_description leg B"
run_oracle_gate ros/verify-joint-states-interop.sh "joint_states leg B"
# `/move_action` last, because it is the most expensive and the least likely to
# be the thing a `cargo fmt` failure was about.
run_oracle_gate ros/verify-move-action-interop.sh "/move_action leg B"

# The summary names the skips, because `all gates passed` meant two different
# things: with the oracle image built it includes upstream's own C++ client
# reaching these endpoints, and without it those legs never ran. A reader
# settling PORTING-PLAN.md's Phase 9 row off a green run cannot tell those
# apart from the status alone.
if [ "${#skipped[@]}" -eq 0 ]; then
  echo "all gates passed"
else
  echo "all gates passed EXCEPT these legs, which were SKIPPED because the oracle"
  echo "image is not built, so upstream's own C++ client never ran against them:"
  printf '  %s\n' "${skipped[@]}"
  echo "Phase 9's completion condition is unmeasured by this run. Build the image"
  echo "with: sg docker -c tools/moveit-oracle/build.sh -- and re-run before citing it."
fi
