#!/bin/bash
# Phase 2's completion condition, as a command rather than a number in a
# report: every committed robot's link FK compared against the C++ oracle
# over N random states at moveit-diff's default 1e-9.
#
# Not a CI step and not a `cargo test`: it needs docker and the
# `moveit-rs/oracle` image, neither of which the workspace test run has.
# `crates/moveit-state/tests/fk_parity.rs` is the committed regression that
# does run everywhere -- it holds four captured cases per robot, which
# catches a port that breaks outright but not one that drifts only on
# configurations nobody captured. This script is what covers that gap, so
# run it after any change to joint kinematics or RobotState::update.
#
#   tools/ci/run-oracle-sweep.sh [CASES] [SEED]
#
# Exits non-zero on the first robot that disagrees.
set -euo pipefail

CASES="${1:-10000}"
SEED="${2:-1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIFF="$REPO_ROOT/target/release/moveit-diff"

# Release, not debug: 10k cases x 4 robots is ~40k FK sweeps per side, and
# the debug build makes the Rust side rather than the oracle the bottleneck.
cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff

for robot in panda fanuc dual_arm_panda pr2; do
  echo "=== $robot ($CASES cases, seed $SEED) ==="
  "$DIFF" \
    --urdf "$REPO_ROOT/fixtures/$robot.urdf" \
    --srdf "$REPO_ROOT/fixtures/$robot.srdf" \
    --cases "$CASES" \
    --seed "$SEED" \
    --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
    | tail -4
done
