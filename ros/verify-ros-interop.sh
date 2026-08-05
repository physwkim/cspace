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
#   - Two live `/move_action` legs over DDS, one of them driven by upstream's
#     own unmodified C++ `MoveGroupInterface` (ros/verify-move-action-interop.sh,
#     called at the end of this script). PORTING-PLAN.md §250 measured that
#     round trip once, by hand; those legs are what re-run it.
#
# What this does NOT check (read this list before wiring `ros/` into CI):
#   - Nothing plans. Both live endpoints are asserted to return a *typed
#     error*, because this workspace has no `moveit_planning::pipeline::
#     Planner` to call (D8/§140.3). No trajectory is produced, so no trajectory
#     is compared against anything; §5 Phase 9's completion condition stays
#     UNMET and these gates are what keep it honestly measured rather than
#     inferred.
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
IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"

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
run "live" bash -c '
  set -e
  cargo build --bin move_group
  ./target/debug/move_group \
    /repo/ros/fixtures/one_joint.urdf /repo/ros/fixtures/one_joint.srdf &
  server_pid=$!
  trap "kill $server_pid 2>/dev/null || true" EXIT
  sleep 3
  out="$(timeout 15 ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "{}")"
  echo "$out"
  # `-E` with a trailing non-digit guard rather than a plain substring: the
  # `grep -q "val=-1"` this replaced also matched `val=-16`, the other code
  # this same handler returns (INVALID_GOAL_CONSTRAINTS, when the request
  # does not convert) -- so it could not tell the two apart, and a handler
  # that answered the conversion error for every request would have passed it.
  echo "$out" | grep -qE "val=99999([^0-9]|$)" || {
    echo "FAIL live round-trip: response did not carry the expected FAILURE (val=99999) error code" >&2
    echo "FAIL upstream answers a null resolvePlanningPipeline with FAILURE in both capabilities" >&2
    echo "FAIL (plan_service_capability.cpp, move_action_capability.cpp); PLANNING_FAILED is for a" >&2
    echo "FAIL pipeline that ran and did not solve, which this port never does." >&2
    exit 1
  }
  echo "$out" | grep -q "no moveit_planning::pipeline::Planner to call yet" || {
    echo "FAIL live round-trip: response did not carry the expected explanatory message" >&2
    exit 1
  }
  # `source` is what separates an answer built by this node from one built
  # anywhere else -- the same assertion the move_action legs make about their
  # own endpoint. It names the endpoint, not the binary: this reply used to
  # carry the binary name and went stale on the wire the moment the binary was
  # renamed (PORTING-PLAN.md §NEW), with nothing looking at it.
  #
  # `\b` and not a trailing `.`: without a right-hand word boundary this would
  # match the `source=...plan_kinematic_path_server` it replaced just as well,
  # and a check that accepts both spellings cannot tell them apart.
  echo "$out" | grep -qE "source=.moveit-ros/plan_kinematic_path\b" || {
    echo "FAIL live round-trip: response did not carry source=moveit-ros/plan_kinematic_path" >&2
    exit 1
  }
'
echo "OK live round-trip: /plan_kinematic_path received a real MotionPlanRequest over DDS and returned the expected typed response"

# `/move_action`, in its own file: it orchestrates three containers and a
# docker network, and it is the only check here that runs upstream's own C++
# client. Last, because it is the most expensive and the least likely to be
# the thing a `cargo fmt` failure was about.
"$REPO_ROOT/ros/verify-move-action-interop.sh"

echo "all gates passed"
