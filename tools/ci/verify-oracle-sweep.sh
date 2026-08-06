#!/bin/bash
# Phase 2's completion condition, as a command rather than a number in a
# report: every committed robot's link FK compared against the C++ oracle
# over N random states at moveit-diff's default 1e-9, and one group's 6xN
# jacobian at 1e-7.
#
# Not a `check-*.sh` step and not a `cargo test`: it needs docker and the
# `moveit-rs/oracle` image, neither of which a CI runner or the workspace
# test run has. `crates/moveit-state/tests/fk_parity.rs` and
# `tests/jacobian.rs` are the committed regressions that do run everywhere
# -- they hold a handful of captured cases per robot, which catches a port
# that breaks outright but not one that drifts only on configurations
# nobody captured. This script is what covers that gap, so run it after
# any change to joint kinematics, RobotState::update, or Posed::jacobian.
#
# Named `verify-*`, run by `tools/ci/verify-all.sh`'s glob rather than by
# a hand-typed pointer in a round brief -- round 17's audit found this
# file sitting outside both the `check-*.sh` and `verify-*.sh` globs with
# nothing invoking it at all, the same never-runs shape §196 and
# `verify-vendored-fixture-tests.sh` both close elsewhere. Wall clock for
# the default 10000 cases is measured in the header comment on
# `CASES_TO_RUN` below, and `verify-all.sh`'s per-round cost absorbs it.
#
# Phase 3's completion condition (`collision: bool` / `distance: f64`) is
# NOT swept here: it is `verify-phase3-collision-sweep.sh`, opt-in, because
# it costs over an hour. Two conditions behind one exit code would also
# make a failure unattributable to either.
#
#   tools/ci/verify-oracle-sweep.sh [CASES] [SEED]
#
# Exits non-zero on the first robot that disagrees.
set -euo pipefail

CASES="${1:-10000}"
SEED="${2:-1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
DIFF="$REPO_ROOT/target/release/moveit-diff"

# `--group` is what turns the jacobian comparison on; without it moveit-diff
# sweeps FK alone. Phase 2 requires both, so every robot names a group here.
# pr2 appears twice on purpose: `base` is the only planar-joint group in the
# fixtures, and the reference-point and mimic handling that a revolute chain
# exercises says nothing about it.
#
# Every entry also runs moveit-diff's §5 Phase 1 clause comparison (link
# count, joint count, group composition, joint limits, mimic -- see
# `compare_model_info_clauses`), once per invocation rather than once per
# case. That is why `prbt` is here even though its `manipulator` chain adds
# no joint type the other entries lack: Phase 1's completion condition names
# panda / prbt / fanuc, and a fixture no script invokes has its clauses
# compared by nothing.
#
# MEASURED wall clock for the whole list at the default 10000 cases: 113s
# (2026-08-05, six entries including the prbt one added with this comment;
# 96-core host, loadavg 18->26 from sibling panels, moveit-diff single-
# threaded). Each entry reports `cases: 20006` = 1 model_info + 5 Phase 1
# clauses + 10000 fk + 10000 jacobian.
#
# For scale: `verify-phase3-collision-sweep.sh` over the same 10000 states
# costs 4815s, ~43x this, which is the measured reason Phase 3 is opt-in and
# this is not.
CASES_TO_RUN=(
  "panda           panda_arm"
  "prbt            manipulator"
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
  # The Phase 1 clause block is echoed as well as the Phase 2 numbers. It is
  # already counted in `cases:`/`failed:` (a run with --group reports
  # 1 model_info + 5 clauses + N fk + N jacobian), so the exit code covered
  # it before -- but only as five anonymous units inside a five-digit total,
  # which is not a per-category result anyone can read off. §5 Phase 1 names
  # five categories, so the five verdicts are printed by name.
  grep -E '^--- Phase 1 clauses|^(link_count|joint_count|group_composition|joint_limits|mimic) |worst jacobian deviation|^cases:|^passed:|^failed:' "$OUT" || true
  if [[ $status -ne 0 ]]; then
    verdict="$(run_verdict "$status" "$OUT" '^failed:')"
    if [[ $verdict == disagreed ]]; then
      echo "--- first 20 disagreements ---" >&2
      grep '^FAIL' "$OUT" | head -20 >&2 || true
      echo "$robot / $group disagreed with the oracle (exit $status)" >&2
    else
      echo "--- last 20 lines of the run ---" >&2
      tail -20 "$OUT" >&2 || true
      echo "$robot / $group did not finish: $verdict -- this is not a disagreement" >&2
    fi
    exit "$status"
  fi
done
