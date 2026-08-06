#!/bin/bash
# Phase 2's third completion condition, as a command rather than a number in
# a report: joint-limit clamping, mimic propagation, and floating/planar
# joint interpolation, each compared against the C++ oracle on every
# committed robot.
#
# The other two Phase 2 conditions are `verify-oracle-sweep.sh` (link FK and
# one group's jacobian, over random states). This is a separate script and a
# separate exit code for the same reason Phase 3's sweep is separate from
# both: two conditions behind one status make a failure unattributable. It is
# also a different *kind* of sweep -- `verify-oracle-sweep.sh` draws random
# states and asks whether the two sides agree on them, which can only find a
# divergence that random sampling happens to land on. Every case here is a
# named boundary value, enumerated from the oracle's own reported bounds:
# exactly at a limit, one ULP inside, one ULP outside, at the wrap point, and
# (for a floating joint's quaternion) at each side of the two different
# epsilons upstream uses on the same value.
#
#   tools/ci/verify-phase2-state-sweep.sh [TOL_INTERPOLATE]
#
# The default tolerance is 0.0 -- bitwise agreement. Clamping and mimic
# propagation are compared exactly always: clamping copies a limit, wrapping
# is an `fmod` plus a conditional add, and mimic propagation is
# `factor * v + offset`. Those are the identical IEEE operations on both
# sides, so any difference at all is a difference in *what* was computed, not
# in rounding. Interpolation takes an argument only so a future divergence
# can be *reported at its measured size*; raising it to make a run pass is
# the thing this script exists to prevent.
#
# Runs every robot before exiting, unlike `verify-oracle-sweep.sh`: a
# condition that names three clauses is not usefully reported as "the first
# robot that failed", and the per-robot counts are the deliverable.
set -euo pipefail

TOL_INTERPOLATE="${1:-0.0}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
DIFF="$REPO_ROOT/target/release/moveit-diff"

# Every committed fixture, because the three clauses reach disjoint code on
# different ones and a missing robot silently removes a clause rather than
# failing it:
#
#   panda           the only floating joint (`virtual_joint`), and the only
#                   mimic whose master is *prismatic* -- upstream's
#                   `enforcePositionBounds` is change-gated for prismatic and
#                   unconditional for revolute, so this fixture and pr2 take
#                   opposite branches on the same clamp-then-propagate case.
#   pr2             the only planar joint, and 19 continuous joints; its
#                   mimic masters are revolute.
#   dual_arm_panda  two mimics rather than one.
#   prbt, fanuc     plain revolute chains -- the case where none of the
#                   special paths applies, which is what says a disagreement
#                   on the others is about the special path.
#   one_robot       the only mimic with `multiplier != 1` or `offset != 0`
#                   (1.5 and 0.1). Every other fixture's mimic is the identity
#                   `1 * v + 0`, where dropping the factor, dropping the
#                   offset, and applying them in the wrong order all produce
#                   the same numbers -- so the arithmetic was covered at the
#                   one point in its parameter space where it cannot be wrong.
#   prbt_pg70       the only SRDF group that names a mimic's *master* without
#                   naming the mimic (Pilz's `gripper` group lists
#                   `prbt_gripper_finger_left_joint` alone). That is the sole
#                   input on which `RobotState::interpolate`'s group overload
#                   and its whole-model overload disagree -- the group form
#                   propagates `group->getMimicJointModels()`, which is empty
#                   here, so the mimic must come out of the *destination*
#                   untouched while its master moves. Every other committed
#                   group either holds both halves of a mimic pair or neither.
ROBOTS=(panda prbt fanuc dual_arm_panda pr2 one_robot prbt_pg70)

# Before comparing anything: confirm the fixtures still are the robots they
# name. Boundary values here are derived from the oracle's own reported
# bounds, so a drifted URDF would move both the case values and the expected
# answers together and still agree.
"$REPO_ROOT/tools/ci/verify-fixture-provenance.sh"

cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff

OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT

failed_robots=()
for robot in "${ROBOTS[@]}"; do
  echo "=== $robot ==="
  # Redirected to a file rather than piped into a filter: a pipe reports the
  # filter's status, which turns a disagreement into a silent pass. The
  # status is captured rather than left to `set -e` so the numbers still get
  # printed on the run that failed.
  status=0
  "$DIFF" \
    --urdf "$REPO_ROOT/fixtures/$robot.urdf" \
    --srdf "$REPO_ROOT/fixtures/$robot.srdf" \
    --state-ops \
    --tol-interpolate "$TOL_INTERPOLATE" \
    --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
    > "$OUT" 2>&1 || status=$?

  # Per-clause counts, the skipped lines (a clause that measured less than it
  # looks like says so rather than reading as a pass), and the verdict.
  grep -E '^(clamping|mimic|interpolation|state_interpolation) |tolerance |SKIPPED |^§5 Phase 2 clause 3' "$OUT" || true
  if [[ $status -ne 0 ]]; then
    verdict="$(run_verdict "$status" "$OUT" '^(clamping|mimic|interpolation|state_interpolation) ')"
    if [[ $verdict == disagreed ]]; then
      echo "--- first 20 disagreements ---" >&2
      grep -E '^  FAIL' "$OUT" | head -20 >&2 || true
      failed_robots+=("$robot")
    else
      # Not added to `failed_robots`: that list feeds a "clause 3 is NOT met"
      # verdict, and a run that never reported cannot support it. Failing here
      # keeps the gate red without the gate claiming a measurement it lacks.
      echo "--- last 20 lines of the run ---" >&2
      tail -20 "$OUT" >&2 || true
      echo "FAIL $robot did not finish: $verdict -- clause 3 is unmeasured on $robot, not unmet" >&2
      exit "$status"
    fi
  fi
done

if [[ ${#failed_robots[@]} -gt 0 ]]; then
  echo "§5 Phase 2 clause 3 is NOT met on: ${failed_robots[*]}" >&2
  exit 1
fi

echo "OK: clamping, mimic propagation and floating/planar interpolation agree with the oracle on all ${#ROBOTS[@]} committed robots"
