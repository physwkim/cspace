#!/bin/bash
# Runs `measure-declaration-audits.py --check` against
# `doc/declaration-audit-coverage.md`, so the per-file audit verdicts move
# when the ported set moves instead of aging in place.
#
# The failure this exists for is silent: a crate adds a `// Ported from
# moveit2 @ <sha>:` line, `measure-port-coverage.py` counts one more file as
# ported, and the declaration-audit table -- which nothing joined to it --
# keeps describing the older set. `MISSING ROW` names the new file and the
# gate fails until somebody rules on it.
#
# Deliberately NOT a `check-*.sh`, for `verify-port-coverage.sh`'s reason: it
# walks `$MOVEIT2_SRC`, an upstream checkout outside this repository and
# absent from a CI runner. Absent checkout is a failure, never a skip.
#
# The pinned SHA is read from `src-digest.sh` rather than copied here: the
# ported set is relative to one upstream revision, so measuring against a
# differently-pinned checkout would move the row set with nothing in the
# output saying the corpus had changed underneath it.
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
  echo "FAIL the ported set this table joins to is relative to the pinned revision." >&2
  exit 1
fi

./tools/ci/measure-declaration-audits.py --upstream "$MOVEIT2_SRC" \
  --check doc/declaration-audit-coverage.md

echo "OK"
