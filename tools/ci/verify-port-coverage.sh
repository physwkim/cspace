#!/bin/bash
# Runs `measure-port-coverage.py --check` against `doc/port-coverage.md`, so
# the 95-row unported table is checked by a command rather than by whoever
# remembers to re-run the instrument.
#
# The table is not prose: `--check` compares its row set against the set the
# instrument computes from the upstream tree and this repo's citation
# headers, and prints `MISSING ROW` / `STALE ROW` for each side of the
# difference. Without a caller that is a mode nobody runs -- the same
# never-runs shape `verify-oracle-sweep.sh`'s header describes, and the one
# `doc/port-coverage.md` itself claims to be immune to. It was immune only
# in the sense that a human ran it once: the merge that brought the table in
# staled four of its rows within the hour, and the instrument said so only
# because that merge was being watched.
#
# Deliberately NOT a `check-*.sh`: it walks `$MOVEIT2_SRC`, an upstream
# checkout outside this repository and absent from a CI runner, exactly like
# `verify-upstream-license-provenance.sh`. Absent checkout is a failure and
# not a skip, for that script's reason: a gate that skips where the input is
# missing reads as coverage while providing none.
#
# The pinned SHA is read from `src-digest.sh` rather than copied here. Every
# number in `doc/port-coverage.md` is relative to one upstream revision, so
# measuring against a differently-pinned checkout would move the counts with
# nothing in the output saying the corpus had changed underneath them.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
if [[ ! -d "$MOVEIT2_SRC" ]]; then
  echo "FAIL $MOVEIT2_SRC is absent -- this gate enumerates the upstream corpus it measures against." >&2
  exit 1
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

if ! head_sha="$(git -C "$MOVEIT2_SRC" rev-parse HEAD 2>/dev/null)"; then
  echo "FAIL $MOVEIT2_SRC is not a git checkout -- the pinned revision cannot be confirmed." >&2
  exit 1
fi
if [[ "$head_sha" != "$ORACLE_MOVEIT2_SHA" ]]; then
  echo "FAIL $MOVEIT2_SRC is at $head_sha, not the pinned $ORACLE_MOVEIT2_SHA." >&2
  echo "FAIL every count in doc/port-coverage.md is relative to the pinned revision." >&2
  exit 1
fi

./tools/ci/measure-port-coverage.py --upstream "$MOVEIT2_SRC" --check doc/port-coverage.md

echo "OK"
