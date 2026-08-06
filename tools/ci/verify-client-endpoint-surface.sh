#!/bin/bash
# Runs `measure-client-endpoint-surface.py --check` against
# `doc/client-endpoint-surface.md`, so Phase 9's requirement -- which
# endpoints the unmodified C++ `MoveGroupInterface` binds, and which of its
# 126 public declarations reach each one -- is re-derived from the pinned
# upstream every run instead of being quoted from a round that measured it
# once.
#
# The failure this exists for is the one Phase 9 kept hitting: the
# requirement was only ever visible one `return` at a time. §226.4 recorded
# the blockage as "the server side", §250.2 split that into four, and running
# it then stopped at a fifth thing none of the four named. Nothing was
# counting the whole set, so every round could only discover the next
# rejection. A gate that fails when the set moves is what makes the count
# survive the round that produced it.
#
# Deliberately NOT a `check-*.sh`, for `verify-upstream-citations.sh`'s
# reason: it reads `$MOVEIT2_SRC`, an upstream checkout outside this
# repository and absent from a CI runner. Absent checkout is a failure, not a
# skip -- a skipped enumeration and an unchanged one print the same thing.
#
# The pinned SHA is read from `src-digest.sh`, not copied here: every hpp line
# number in the checked-in table is relative to one upstream revision, so
# checking the table against a differently-pinned checkout would move rows
# with nothing in the output saying the reference had changed underneath them.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
if [[ ! -d "$MOVEIT2_SRC" ]]; then
  echo "FAIL $MOVEIT2_SRC is absent -- this gate enumerates the upstream client it measures." >&2
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
  echo "FAIL every hpp line in doc/client-endpoint-surface.md is relative to the" >&2
  echo "FAIL pinned revision; rows checked against another one move for no reason." >&2
  exit 1
fi

./tools/ci/measure-client-endpoint-surface.py --upstream "$MOVEIT2_SRC" \
  --check doc/client-endpoint-surface.md

echo "OK"
