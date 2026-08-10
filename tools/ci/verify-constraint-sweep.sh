#!/bin/bash
# Phase 5's first completion condition, as a command rather than a number in
# a report: 2,000 generated constraint *combinations* per robot, each
# decided by `cspace-constraints`' `decide()` and by the C++ oracle's
# `kinematic_constraints::KinematicConstraintSet::decide`, compared
# constraint by constraint (`satisfied` always, `distance` at
# `--tol-constraints`' 1e-9 default -- except `visibility_cone`'s distance,
# see `compare_constraints`' doc comment in `tools/moveit-diff/src/main.rs`
# for what that one skip gives up and why no tolerance closes it).
#
# `moveit-diff --constraints 2000` has existed since round 4 and nothing ran
# it: it was a hand-typed command in round briefs and in `PORTING-PLAN.md`
# prose, which is the same never-runs shape `verify-oracle-sweep.sh`'s own
# header describes for Phase 2. A completion condition whose only instrument
# is a command someone has to remember is not gated; this script is the
# gate.
#
# Deliberately NOT a `check-*.sh`: it needs docker, the `moveit-rs/oracle`
# image and the gitignored `third_party/` tree (via
# `verify-fixture-provenance.sh`), none of which a CI runner has. Named
# `verify-*.sh` so `tools/ci/verify-all.sh`'s glob picks it up with no list
# to keep in sync.
#
# Cost, measured against the current tree at the defaults below: panda 4.8s,
# fanuc 6.1s, pr2 13.9s, dual_arm_panda 5.7s -- ~31s wall for all four,
# which is well inside what `verify-all.sh`'s per-round cost already absorbs
# (`verify-oracle-sweep.sh` alone is ~2m24s), so there is no extra opt-in
# knob here. Raising `CASES` is what makes it slow: pr2's per-case cost is
# dominated by the `visibility_cone` shape's real FCL cone-vs-robot check.
#
#   tools/ci/verify-constraint-sweep.sh [CASES] [SEED] [POOL]
#
# CASES is the number of constraint combinations per robot (default 2000,
# the completion condition's own number). POOL is how many random states the
# combinations are drawn from and cycled through (default 50); it also sizes
# the fk comparison that runs alongside, which is why it is not 1.
#
# Exits non-zero on the first robot that disagrees.
set -euo pipefail

CASES="${1:-2000}"
SEED="${2:-7}"
POOL="${3:-50}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
require_caller_tree "$REPO_ROOT"

# `--constraints 0` is not an error to `moveit-diff` -- it is how every other
# caller (`verify-oracle-sweep.sh`) says "no constraint sweep", so the tool
# cannot reject it the way `Config` rejects `--cases 0`. That leaves this gate
# as the only place that knows a zero-combination run is meaningless: without
# this guard `verify-constraint-sweep.sh 0` exits 0 having printed an empty
# composition table and `passed: 2` (the model_info and fk baselines), which
# is the one outcome a gate must never spell the same way as success.
require_nonempty "$CASES" "constraint combinations to compare"
DIFF="$REPO_ROOT/target/release/moveit-diff"

# Every committed robot, not a representative one. The four differ in what
# this sweep can even reach: only pr2 has parry-representable link collision
# geometry, so it is the only fixture whose `visibility_cone` cases take
# `decide_cone`'s near/collide branch at all (see `build_constraint_case`'s
# doc comment); dual_arm_panda is the only one with two arms, so it is the
# only one whose link cycling crosses a second kinematic chain.
ROBOTS=(panda fanuc pr2 dual_arm_panda)

# Same reason as `verify-oracle-sweep.sh`: a sweep that agrees with the
# oracle on a drifted fixture proves both sides read the same file, not that
# either matches upstream.
"$REPO_ROOT/tools/ci/verify-fixture-provenance.sh"

cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff

OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT

for robot in "${ROBOTS[@]}"; do
  echo "=== $robot ($CASES combinations, seed $SEED, state pool $POOL) ==="
  # Redirected rather than piped, and the status captured rather than left
  # to `set -e`, for the same two reasons `verify-oracle-sweep.sh` gives:
  # a pipe reports the filter's status (turning a disagreement into a silent
  # pass), and aborting before the summary prints leaves the caller with an
  # exit code and no numbers.
  status=0
  "$DIFF" \
    --urdf "$REPO_ROOT/fixtures/$robot.urdf" \
    --srdf "$REPO_ROOT/fixtures/$robot.srdf" \
    --cases "$POOL" \
    --seed "$SEED" \
    --constraints "$CASES" \
    --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
    > "$OUT" 2>&1 || status=$?

  # The composition table, not just the totals: `--constraints N` passing
  # says nothing about whether the N cases were N variations of one kind.
  # Printing the breakdown is what makes the completion condition's "all
  # four kinds and their combinations" readable from the gate's own output.
  composition="$(sed -n '/^constraint combinations by composition:/,/^constraint satisfied/p' "$OUT")"
  printf '%s\n' "$composition"
  grep -E '^cases:|^passed:|^failed:' "$OUT" || true

  if [[ $status -ne 0 ]]; then
    verdict="$(run_verdict "$status" "$OUT" '^failed:')"
    if [[ $verdict == disagreed ]]; then
      echo "--- first 20 disagreements ---" >&2
      grep '^FAIL' "$OUT" | head -20 >&2 || true
      echo "$robot disagreed with the oracle (exit $status)" >&2
    else
      echo "--- last 20 lines of the run ---" >&2
      tail -20 "$OUT" >&2 || true
      echo "$robot did not finish: $verdict -- this is not a disagreement" >&2
    fi
    exit "$status"
  fi

  # The table is printed above for a reader; this reads it back for the gate.
  # `passed: N` counts verdicts, and the constraint sweep is only part of them
  # (`cases:` is CASES + 1 model_info + POOL fk), so agreement on the total is
  # not evidence that the combinations ran: a generator that emitted half the
  # combinations it was asked for would still pass every comparison it made.
  # Summing the composition rows is the one number in this output that has to
  # equal what the run asked for.
  combinations="$(printf '%s\n' "$composition" \
    | sed -n 's/^  .*: \([0-9][0-9]*\) cases$/\1/p' \
    | awk '{ s += $1 } END { print s + 0 }')"
  if [[ "$combinations" -ne "$CASES" ]]; then
    echo "FAIL $robot: the composition table accounts for $combinations combinations, not the $CASES asked for." >&2
    echo "FAIL every generated combination must appear in the table, so the two disagreeing means the sweep did not run what it reported." >&2
    exit 1
  fi
done
