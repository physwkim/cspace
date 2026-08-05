#!/bin/bash
# Resolves every `<file>.cpp:NNN` / `.hpp` / `.h` citation in the tracked
# `.md` and `.rs` files against the pinned upstream checkout, and fails on the
# ones that no longer land where the citing text says they do.
#
# This is the half `tools/ci/check-citation-drift.py` names in its own header
# as out of scope ("not resolvable without a local upstream checkout and are
# not covered here"), and it is where the drift is. Hand-checking five upstream
# citations during one merge found three wrong, one of them a citation that
# round had just corrected: `planning_scene.cpp:2451-2490` for
# `getCostSources`'s trajectory pair, whose two overloads are `2451-2455` and
# `2457-2491`. Three more `2451-2490`s were in the tree behind it, and nothing
# mechanical was looking at any of them.
#
# Deliberately NOT a `check-*.sh`: that glob is what `.github/workflows/ci.yml`
# runs, on a bare runner with nothing but this repository. This gate walks
# `$MOVEIT2_SRC`, an upstream checkout outside it -- the same reason
# `verify-port-coverage.sh` and `verify-upstream-license-provenance.sh` carry
# the `verify-` name. `tools/ci/verify-all.sh`'s glob picks it up.
#
# Absent checkout SKIPs, loudly, in `verify-mpr-vs-epa.sh`'s shape: a silent
# skip is indistinguishable from a pass. A checkout at the WRONG revision is a
# hard failure instead, and that asymmetry is the point -- every line number in
# this corpus is relative to one upstream revision, so checking a citation
# against a different one produces confident verdicts about a file nobody
# cited. That is worse than not checking, because it reads as coverage and
# argues for edits.
#
#   tools/ci/verify-upstream-citations.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
if [[ ! -d "$MOVEIT2_SRC" ]]; then
  echo "SKIP $MOVEIT2_SRC not present -- no upstream citation was resolved."
  echo "SKIP this is not a pass; clone https://github.com/moveit/moveit2 there to cover them."
  exit 0
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

if ! head_sha="$(git -C "$MOVEIT2_SRC" rev-parse HEAD 2>/dev/null)"; then
  echo "FAIL $MOVEIT2_SRC is not a git checkout -- the pinned revision cannot be confirmed." >&2
  exit 1
fi
if [[ "$head_sha" != "$ORACLE_MOVEIT2_SHA" ]]; then
  echo "FAIL $MOVEIT2_SRC is at $head_sha, not the pinned $ORACLE_MOVEIT2_SHA." >&2
  echo "FAIL every upstream line number in this repository is relative to the pinned" >&2
  echo "FAIL revision; a citation checked against another one is worse than unchecked." >&2
  exit 1
fi

./tools/ci/measure-upstream-citations.py --upstream "$MOVEIT2_SRC"
