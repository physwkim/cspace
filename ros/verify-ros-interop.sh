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
#
# What this does NOT check (read this list before wiring `ros/` into CI):
#   - No live ROS 2 graph: no node is ever spun up, no topic/service/action
#     is published or called against a real moveit2 or rclrs process. Every
#     test in ros/moveit-ros/src/**/*.rs constructs `r2r`-generated message
#     structs in-process and converts them -- it never round-trips a message
#     through the DDS middleware. Wire-format compatibility with a real
#     moveit2 node is unverified by this script.
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

# Only the unit-test phase's own summary line: doctests print a second,
# separate "test result:" line, and a unit-test run filtered down to
# nothing must not hide behind an unrelated doctest count that happens to
# be nonzero.
#
# `|| true` on both assignments below: under this script's own `set -e` and
# `pipefail`, `grep`'s legitimate "no match" exit (1) propagates to the whole
# pipeline and aborts the assignment before the `-z` check that follows it
# ever runs -- the same `test_status=$?` shape 48ef7ce closed, reappearing
# through pipefail instead of `$?`. Round 18's sweep found it: a unit-test run
# filtered down to nothing produced no diagnostic at all, not even this
# file's own "could not find..." message below, because the script had
# already died one line above it.
unit_summary="$(sed -n '1,/^   Doc-tests/p' <<<"$test_output" | grep -E '^test result: ' | tail -1 || true)"
if [[ -z "$unit_summary" ]]; then
  echo "FAIL could not find the unit-test 'test result:' line in cargo test's output -- nothing was checked." >&2
  exit 1
fi
actual_tests="$(grep -oE '[0-9]+ passed' <<<"$unit_summary" | grep -oE '^[0-9]+' || true)"
if [[ -z "$actual_tests" ]]; then
  echo "FAIL could not parse a passing-test count out of: $unit_summary" >&2
  exit 1
fi
if [[ "$actual_tests" -ne "$expected_tests" ]]; then
  echo "FAIL cargo test reported $actual_tests passing unit test(s) but ros/moveit-ros/src has $expected_tests '#[test]' function(s)." >&2
  echo "FAIL a stray #[cfg], a filter, or a renamed module silently dropped $((expected_tests - actual_tests)) of them from the run." >&2
  exit 1
fi
echo "OK $actual_tests/$expected_tests source-declared unit tests actually ran"

run "doc" bash -c "cargo doc --no-deps"

echo "all gates passed"
