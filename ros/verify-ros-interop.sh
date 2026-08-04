#!/bin/bash
# Builds ros/moveit-ros and runs fmt/clippy/test inside the ROS 2 Rolling +
# Rust image (ros/Dockerfile). Named `verify-*`, not `check-*`, on purpose:
# it needs docker, and tools/ci/'s `check-*.sh` glob is run by CI runners
# that don't have it (PORTING-PLAN.md §129.4).
#
# Not run automatically by anything yet -- there is no CI hook for `ros/`
# this round (D5: ros/moveit-ros is outside the root workspace, so nothing
# in tools/ci/ walks into it). This script is how a human (or a future CI
# job explicitly opted into needing docker) reproduces this round's gate
# results locally.
#
# What this checks (all inside ros/Dockerfile's image, against
# ros/moveit-ros's own `[workspace]`, not the root workspace):
#   - `cargo fmt --check`
#   - `cargo clippy --all-targets -- -D warnings`
#   - `cargo test` (unit tests + doctests for ros/moveit-ros; nextest is not
#     installed in this image -- see the `run "test"` line below)
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
run "test" bash -c "cargo test"
run "doc" bash -c "cargo doc --no-deps"

echo "all gates passed"
