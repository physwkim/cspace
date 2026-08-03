#!/bin/bash
# Phase 2's completion condition, as a command rather than a number in a
# report: every committed robot's link FK compared against the C++ oracle
# over N random states at moveit-diff's default 1e-9, and one group's 6xN
# jacobian at 1e-7.
#
# Not a CI step and not a `cargo test`: it needs docker and the
# `moveit-rs/oracle` image, neither of which the workspace test run has.
# `crates/moveit-state/tests/fk_parity.rs` and `tests/jacobian.rs` are the
# committed regressions that do run everywhere -- they hold a handful of
# captured cases per robot, which catches a port that breaks outright but not
# one that drifts only on configurations nobody captured. This script is what
# covers that gap, so run it after any change to joint kinematics,
# RobotState::update, or Posed::jacobian.
#
#   tools/ci/run-oracle-sweep.sh [CASES] [SEED]
#
# Exits non-zero on the first robot that disagrees.
set -euo pipefail

CASES="${1:-10000}"
SEED="${2:-1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIFF="$REPO_ROOT/target/release/moveit-diff"

# `--group` is what turns the jacobian comparison on; without it moveit-diff
# sweeps FK alone. Phase 2 requires both, so every robot names a group here.
# pr2 appears twice on purpose: `base` is the only planar-joint group in the
# fixtures, and the reference-point and mimic handling that a revolute chain
# exercises says nothing about it.
CASES_TO_RUN=(
  "panda           panda_arm"
  "fanuc           manipulator"
  "dual_arm_panda  left_panda_arm"
  "pr2             right_arm"
  "pr2             base"
)

# Before comparing anything: confirm the fixtures still are the robots they
# name. A sweep that agrees with the oracle on a drifted `fixtures/panda.urdf`
# proves both sides read the same file, not that either matches upstream panda.
# This runs here rather than in the `check-*.sh` set because it needs
# `third_party/`, which this script already requires and CI does not have.
"$REPO_ROOT/tools/ci/verify-fixture-provenance.sh"

# Release, not debug: 10k cases x 5 sweeps is ~50k FK+jacobian evaluations per
# side, and the debug build makes the Rust side rather than the oracle the
# bottleneck.
cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff

OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT

for entry in "${CASES_TO_RUN[@]}"; do
  read -r robot group <<<"$entry"
  echo "=== $robot / $group ($CASES cases, seed $SEED) ==="
  # Redirected to a file rather than piped into a filter: a pipe reports the
  # filter's status, which turns a disagreement into a silent pass. The status
  # is captured instead of being left to `set -e` so the summary still gets
  # printed on the run that failed -- aborting before it is what would leave
  # the caller with an exit code and no numbers.
  status=0
  "$DIFF" \
    --urdf "$REPO_ROOT/fixtures/$robot.urdf" \
    --srdf "$REPO_ROOT/fixtures/$robot.srdf" \
    --cases "$CASES" \
    --seed "$SEED" \
    --group "$group" \
    --tol-jacobian 1e-7 \
    --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
    > "$OUT" 2>&1 || status=$?
  grep -E 'worst jacobian deviation|^cases:|^passed:|^failed:' "$OUT" || true
  if [[ $status -ne 0 ]]; then
    echo "--- first 20 disagreements ---" >&2
    grep '^FAIL' "$OUT" | head -20 >&2 || true
    echo "$robot / $group disagreed with the oracle (exit $status)" >&2
    exit "$status"
  fi
done
