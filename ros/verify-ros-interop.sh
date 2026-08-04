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

echo "all gates passed"
